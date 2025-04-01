// Copyright 2025 PAL Robotics, S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "remap_plugin_query/plugin_query.hpp"

namespace remap
{
namespace plugins
{
PluginQuery::PluginQuery()
: SemanticPlugin() {}

PluginQuery::PluginQuery(
  std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
  std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register)
: SemanticPlugin(semantic_map, regions_register) {}

PluginQuery::~PluginQuery()
{
  semantic_map_.reset();
  regions_register_.reset();
}

void PluginQuery::initialize()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "PluginQuery initializing");

  query_executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
  query_node_ = rclcpp::Node::make_shared("remap_query_node");

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*query_node_);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*query_node_);

  query_server_ = node_ptr_->create_service<remap_msgs::srv::Query>(
    "/remap/query", 
    std::bind(&PluginQuery::queryCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  remove_query_server_ = node_ptr_->create_service<remap_msgs::srv::RemoveQuery>(
    "/remap/remove_query",
    std::bind(&PluginQuery::removeQueryCallback, this, 
      std::placeholders::_1, std::placeholders::_2));
  query_client_ = query_node_->create_client<kb_msgs::srv::Query>("/kb/query");
}

std::vector<std::string> PluginQuery::split(
  std::string s,
  const std::string & delimiter)
{
  // From https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
  std::vector<std::string> tokens;
  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
      token = s.substr(0, pos);
      tokens.push_back(token);
      s.erase(0, pos + delimiter.length());
  }
  tokens.push_back(s);

  return tokens;
}

std::shared_ptr<kb_msgs::srv::Query::Response> PluginQuery::performQuery(
  Query & query) {

  kb_msgs::srv::Query::Request kb_query;
  kb_query.patterns = query.patterns_;
  kb_query.vars = query.vars_;
  kb_query.models = query.models_;

  auto kb_query_result = query_client_->async_send_request(std::make_shared<kb_msgs::srv::Query::Request>(kb_query));
  std::shared_ptr<kb_msgs::srv::Query::Response> kb_query_result_raw;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(query_node_, kb_query_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    // we go over the results
    kb_query_result_raw = kb_query_result.get(); 
    std::string query_results = kb_query_result_raw->json;
    if (query_results.size() > 2)
    {
      auto query_results_vec = split(query_results.substr(1, query_results.size() - 2), ",");
      std::map<std::string, pcl::PointCloud<pcl::PointXYZI>> spatial_query_results;
      for (auto query_result : query_results_vec)
      {
        auto query_result_json = nlohmann::json::parse(query_result);
        for (const auto & elem : query_result_json.items())
        {
          if (spatial_query_results.find(elem.key()) == spatial_query_results.end())
          {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            spatial_query_results[elem.key()] = cloud;
          }
          semantic_map_->getEntity(elem.value(), *regions_register_, spatial_query_results[elem.key()]);
        }
      }
      for (const auto & spatial_query_result : spatial_query_results)
      {
        query.publish(spatial_query_result.first, spatial_query_result.second, semantic_map_->getFixedFrame());
        if (query.publish_tf_) {
          geometry_msgs::msg::TransformStamped t;
          auto centroid_it = query.centroids_.find(spatial_query_result.first);
          if (centroid_it == query.centroids_.end()) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Centroid not found for %s", spatial_query_result.first.c_str());
            continue;
          }
          t.header.stamp = node_ptr_->get_clock()->now();
          t.child_frame_id = query.id_ + "/" + spatial_query_result.first;
          t.header.frame_id = semantic_map_->getFixedFrame();
          t.transform.translation.x = centroid_it->second[0];
          t.transform.translation.y = centroid_it->second[1];
          t.transform.translation.z = centroid_it->second[2];
          t.transform.rotation.x = 0;
          t.transform.rotation.y = 0;
          t.transform.rotation.z = 0;
          t.transform.rotation.w = 1;
          if (query.dynamic_) {
            tf_broadcaster_->sendTransform(t);
          }
          else {
            static_tf_broadcaster_->sendTransform(t);
          }
        }
      }
    }
  } else {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to call the kb query");
    return nullptr;
  }

  query.last_execution_ = node_ptr_->get_clock()->now();

  return kb_query_result_raw;
}

void PluginQuery::queryCallback(
  const std::shared_ptr<remap_msgs::srv::Query::Request> req,
  const std::shared_ptr<remap_msgs::srv::Query::Response> res)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Received reMap query");

  if ((req->frequency.sec < 0) || ((req->frequency.sec == 0) && (req->frequency.nanosec == 0))) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Frequency is not valid, aborting");
    res->success = false;
    return;
  }

  auto query = Query(
    node_ptr_, req->id, req->patterns, req->vars,
    req->models, req->dynamic, req->duration, req->frequency, req->publish_tf);

  if (!query_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "knowledge base not available, aborting");
    res->success = false;
    return;
  }

  auto kb_query_result_raw = performQuery(query);

  if (query.dynamic_) {
    queries_[req->id] = query;
  } else {
    static_queries_[req->id] = query;
  }

  if (kb_query_result_raw)
  {
    res->success = kb_query_result_raw->success;
    res->json = kb_query_result_raw->json;
    res->error_msg = kb_query_result_raw->error_msg;

    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Query result: " << res->json);
  } else {
    RCLCPP_WARN(node_ptr_->get_logger(), "knowledge base not available, aborting");
    res->success = false;
  }
}

void PluginQuery::removeQueryCallback(
    const std::shared_ptr<remap_msgs::srv::RemoveQuery::Request> req,
    const std::shared_ptr<remap_msgs::srv::RemoveQuery::Response> res) {
  auto query_it = queries_.find(req->id);

  if (query_it == queries_.end()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Query %s not found", req->id.c_str());
    res->success = false;
    return;
  }

  if (query_it->second.req_duration_.seconds() >= 0) {
    res->exec_left = (query_it->second.req_time_ + query_it->second.req_duration_) - node_ptr_->get_clock()->now();
  } else {
    res->exec_left = rclcpp::Duration(-1, 0);
  }

  queries_.erase(query_it);
  RCLCPP_INFO(node_ptr_->get_logger(), "Removed query %s", req->id.c_str());

  res->success = true;
}

void PluginQuery::run()
{
  if (!query_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "knowledge base not available, aborting");
    return;
  }

  std::vector<std::string> queries_to_remove;

  for (auto & query : queries_) {
    if (query.second.dynamic_) {
      if (((query.second.req_time_ + query.second.req_duration_) < node_ptr_->get_clock()->now())
       && (query.second.req_duration_.seconds() > 0)) {
        queries_to_remove.push_back(query.first);
        RCLCPP_WARN(node_ptr_->get_logger(), "Completed query %s", query.first.c_str());
      } else {
        if ((query.second.last_execution_ + query.second.frequency_)
         < node_ptr_->get_clock()->now()) {
          performQuery(query.second);
        }
      }
    }
  }

  for (const auto & query : queries_to_remove) {
    queries_.erase(query);
  }
}

}  // namespace plugins
}  // namespace remap

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(remap::plugins::PluginQuery, remap::plugins::PluginBase)

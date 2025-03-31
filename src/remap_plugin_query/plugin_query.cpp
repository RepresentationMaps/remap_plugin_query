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

  query_server_ = node_ptr_->create_service<remap_msgs::srv::Query>(
    "/remap/query", std::bind(&PluginQuery::queryCallback, this, std::placeholders::_1, std::placeholders::_2));
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

void PluginQuery::queryCallback(
  const std::shared_ptr<remap_msgs::srv::Query::Request> req,
  const std::shared_ptr<remap_msgs::srv::Query::Response> res)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Received reMap query");

  kb_msgs::srv::Query::Request kb_query;
  kb_query.patterns = req->patterns;
  kb_query.vars = req->vars;
  kb_query.models = req->models;

  queries_[req->id] = Query(req->id, req->patterns, req->duration);

  // forwarding the query to the knowledge base
  if (!query_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "knowledge base not available, aborting");
    res->success = false;
    return;
  }

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
      std::map<std::string, pcl::PointCloud<pcl::PointXYZ>> spatial_query_results;
      for (auto query_result : query_results_vec)
      {
        auto query_result_json = nlohmann::json::parse(query_result);
        for (const auto & elem : query_result_json.items())
        {
          // std::cout<<elem.key()<<"\t"<<elem.value()<<std::endl;
          if (spatial_query_results.find(elem.key()) == spatial_query_results.end())
          {

            pcl::PointCloud<pcl::PointXYZ> cloud;
            spatial_query_results[elem.key()] = cloud;
          }
          semantic_map_->getEntity(elem.value(), *regions_register_, spatial_query_results[elem.key()]);
        }
      }
      for (const auto & spatial_query_result : spatial_query_results)
      {
        if (query_pubs_.find(req->id + "/" + spatial_query_result.first) == query_pubs_.end())
        {
          rclcpp::QoS pub_qos(1);
          pub_qos.reliable();
          pub_qos.transient_local();

          query_pubs_[req->id + "/" + spatial_query_result.first] = query_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/remap/query/results/" + req->id + "/" + spatial_query_result.first, pub_qos);
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(spatial_query_result.second, cloud_msg);
        cloud_msg.header.stamp = node_ptr_->now();
        cloud_msg.header.frame_id = semantic_map_->getFixedFrame();
        query_pubs_[req->id + "/" + spatial_query_result.first]->publish(cloud_msg);
      }
    }
  } else {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to call the kb query");
    res->success = false;
    return;
  }

  if (kb_query_result_raw)
  {
    res->success = kb_query_result_raw->success;
    res->json = kb_query_result_raw->json;
    res->error_msg = kb_query_result_raw->error_msg;

    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Query result: " << res->json);
  }
}

void PluginQuery::run()
{
}

}  // namespace plugins
}  // namespace remap

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(remap::plugins::PluginQuery, remap::plugins::PluginBase)

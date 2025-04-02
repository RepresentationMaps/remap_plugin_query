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

#ifndef REMAP_PLUGIN_QUERY__PLUGIN_QUERY_HPP_
#define REMAP_PLUGIN_QUERY__PLUGIN_QUERY_HPP_

#include <pcl/common/centroid.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kb_msgs/srv/query.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <remap_msgs/srv/query.hpp>
#include <remap_msgs/srv/remove_query.hpp>
#include <remap_plugin_base/plugin_base.hpp>
#include <remap_plugin_base/semantic_plugin.hpp>

namespace remap
{
namespace plugins
{
struct Query
{
  std::shared_ptr<rclcpp::Node> node_ptr_;

  std::string id_;

  std::vector<std::string> patterns_;
  std::vector<std::string> vars_;
  std::vector<std::string> models_;

  bool dynamic_;

  rclcpp::Duration req_duration_;
  rclcpp::Duration frequency_;
  rclcpp::Time req_time_;
  rclcpp::Time last_execution_;

  bool publish_tf_;
  bool executed_;
  std::map<std::string, Eigen::Vector4d> centroids_;

  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> query_pubs_;

  Query()
  : req_duration_(0, 0), frequency_(0, 0) {}

  Query(
    const std::shared_ptr<rclcpp::Node> node_ptr,
    const std::string & id,
    const std::vector<std::string> & patterns,
    const std::vector<std::string> & vars,
    const std::vector<std::string> & models,
    const bool & dynamic,
    const rclcpp::Duration & req_duration,
    const rclcpp::Duration & frequency,
    const bool & publish_tf = false)
  : node_ptr_(node_ptr),
    id_(id),
    patterns_(patterns),
    vars_(vars),
    models_(models),
    dynamic_(dynamic),
    req_duration_(req_duration),
    frequency_(frequency),
    req_time_(node_ptr_->get_clock()->now()),
    last_execution_(node_ptr_->get_clock()->now()),
    publish_tf_(publish_tf),
    executed_(false) {}

  ~Query()
  {
    query_pubs_.clear();
  }

  void addPublisher(const std::string & variable)
  {
    rclcpp::QoS pub_qos(1);
    pub_qos.reliable();
    if (!dynamic_) {
      pub_qos.transient_local();
    }

    query_pubs_[variable] = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/remap/query/results/" + id_ + "/" + variable, pub_qos);
  }

  void publish(
    const std::string & variable,
    const pcl::PointCloud<pcl::PointXYZI> & cloud,
    const std::string & frame_id)
  {
    if (cloud.empty()) {
      return;
    }
    if (query_pubs_.find(variable) == query_pubs_.end()) {
      addPublisher(variable);
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = node_ptr_->get_clock()->now();
    cloud_msg.header.frame_id = frame_id;
    query_pubs_[variable]->publish(cloud_msg);
    if (publish_tf_) {
      centroids_[variable] = Eigen::Vector4d::Zero();
      pcl::compute3DCentroid(cloud, centroids_[variable]);
    }
  }
};

class PluginQuery : public SemanticPlugin
{
private:
  rclcpp::Executor::SharedPtr query_executor_;
  rclcpp::Node::SharedPtr query_node_;

  rclcpp::Service<remap_msgs::srv::Query>::SharedPtr query_server_;
  rclcpp::Service<remap_msgs::srv::RemoveQuery>::SharedPtr remove_query_server_;
  std::map<std::string,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> static_query_pubs_;
  rclcpp::Client<kb_msgs::srv::Query>::SharedPtr query_client_;

  std::map<std::string, Query> queries_;
  std::map<std::string, Query> static_queries_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  void queryCallback(
    const std::shared_ptr<remap_msgs::srv::Query::Request> req,
    const std::shared_ptr<remap_msgs::srv::Query::Response> res);

  void removeQueryCallback(
    const std::shared_ptr<remap_msgs::srv::RemoveQuery::Request> req,
    const std::shared_ptr<remap_msgs::srv::RemoveQuery::Response> res);

  std::vector<std::string> split(
    std::string s,
    const std::string & delimiter);

  std::shared_ptr<kb_msgs::srv::Query::Response> performQuery(Query & query);

  Query loadQueryFromYAML(const std::string & file_path);

public:
  PluginQuery();
  PluginQuery(
    std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
    std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register);
  ~PluginQuery();
  void run() override;
  void initialize() override;
};
}    // namespace plugins
}  // namespace remap
#endif  // REMAP_PLUGIN_QUERY__PLUGIN_QUERY_HPP_

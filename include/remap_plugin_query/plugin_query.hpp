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

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <kb_msgs/srv/query.hpp>

#include <remap_msgs/srv/query.hpp>
#include <remap_plugin_base/plugin_base.hpp>
#include <remap_plugin_base/semantic_plugin.hpp>

#include <nlohmann/json.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace remap
{
namespace plugins
{
struct Query {
  std::string id_;
  std::vector<std::string> patterns_;
  rclcpp::Time req_time_;
  rclcpp::Duration req_duration_;
  bool dynamic_;
  Query(): req_duration_(0, 0){}

  Query(
    const std::string & id,
    const std::vector<std::string> & patterns,
    const rclcpp::Duration & req_duration):
  id_(id), patterns_(patterns), req_duration_(req_duration) {
    req_time_ = rclcpp::Clock().now();
  }
};

class PluginQuery : public SemanticPlugin
{
private:
  rclcpp::Executor::SharedPtr query_executor_;
  rclcpp::Node::SharedPtr query_node_;

  rclcpp::Service<remap_msgs::srv::Query>::SharedPtr query_server_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> query_pubs_;
  rclcpp::Client<kb_msgs::srv::Query>::SharedPtr query_client_;

  std::map<std::string, Query> queries_;

  void queryCallback(
    const std::shared_ptr<remap_msgs::srv::Query::Request> req,
    const std::shared_ptr<remap_msgs::srv::Query::Response> res);

  std::vector<std::string> split(
    std::string s,
    const std::string & delimiter);
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
#endif  // REMAP_PLUGIN_FACES__PLUGIN_FACES_HPP_

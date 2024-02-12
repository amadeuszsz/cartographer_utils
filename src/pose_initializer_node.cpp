// Copyright 2024 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cartographer_utils/pose_initializer_node.hpp"


namespace cartographer_utils
{
namespace pose_initializer
{
using namespace std::literals::chrono_literals;

PoseInitializerNode::PoseInitializerNode(const rclcpp::NodeOptions & options)
:  Node("pose_initializer", options)
{
  configuration_directory_ = this->declare_parameter("configuration_directory", "/path/to/dir");
  configuration_basename_ = this->declare_parameter("configuration_basename", "config.lua");

  initial_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "/initialpose", 1,
    std::bind(&PoseInitializerNode::initialPoseCallback, this, std::placeholders::_1));
  get_trajectory_states_client_ = this->create_client<GetTrajectoryStates>(
    "get_trajectory_states");
  finish_trajectory_client_ = this->create_client<FinishTrajectory>(
    "finish_trajectory");
  start_trajectory_client_ = this->create_client<StartTrajectory>(
    "start_trajectory");

  waitForService<GetTrajectoryStates>(get_trajectory_states_client_);
  waitForService<FinishTrajectory>(finish_trajectory_client_);
  waitForService<StartTrajectory>(start_trajectory_client_);
}

void PoseInitializerNode::initialPoseCallback(
  const PoseWithCovarianceStamped::SharedPtr msg)
{
  auto req = std::make_shared<cartographer_ros_msgs::srv::GetTrajectoryStates::Request>();

  get_trajectory_states_client_->async_send_request(
    req,
    [this, msg](
      rclcpp::Client<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedFuture result) {
      const auto & response = result.get();
      auto trajectory_id_first = response->trajectory_states.trajectory_id.front();
      auto trajectory_id_last = response->trajectory_states.trajectory_id.back();

      auto req = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
      req->trajectory_id = trajectory_id_last;

      finish_trajectory_client_->async_send_request(
        req,
        [this, msg,
        trajectory_id_first]([[maybe_unused]] rclcpp::Client<FinishTrajectory>::SharedFuture result)
        {
          auto req = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
          req->configuration_directory = configuration_directory_;
          req->configuration_basename = configuration_basename_;
          req->use_initial_pose = true;
          req->relative_to_trajectory_id = trajectory_id_first;
          req->initial_pose.position.x = msg->pose.pose.position.x;
          req->initial_pose.position.y = msg->pose.pose.position.y;
          req->initial_pose.position.z = msg->pose.pose.position.z;
          req->initial_pose.orientation.x = msg->pose.pose.orientation.x;
          req->initial_pose.orientation.y = msg->pose.pose.orientation.y;
          req->initial_pose.orientation.z = msg->pose.pose.orientation.z;
          req->initial_pose.orientation.w = msg->pose.pose.orientation.w;
          start_trajectory_client_->async_send_request(
            req,
            [this]([[maybe_unused]] rclcpp::Client<StartTrajectory>::SharedFuture result) {});
        });
    });
}

template<typename T>
void PoseInitializerNode::waitForService(const typename rclcpp::Client<T>::SharedPtr client)
{
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for service %s.", client->get_service_name());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for service %s connection...", client->get_service_name());
  }
}

}  // namespace pose_initializer
}  // namespace cartographer_utils

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cartographer_utils::pose_initializer::PoseInitializerNode)

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

#ifndef CARTOGRAPHER_UTILS__POSE_INITIALIZER_NODE_HPP_
#define CARTOGRAPHER_UTILS__POSE_INITIALIZER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <cartographer_ros_msgs/srv/get_trajectory_states.hpp>
#include <cartographer_ros_msgs/srv/finish_trajectory.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "cartographer_utils/visibility_control.hpp"

namespace cartographer_utils
{
namespace pose_initializer
{
using geometry_msgs::msg::PoseWithCovarianceStamped;
using cartographer_ros_msgs::srv::GetTrajectoryStates;
using cartographer_ros_msgs::srv::FinishTrajectory;
using cartographer_ros_msgs::srv::StartTrajectory;

class CARTOGRAPHER_UTILS_PUBLIC PoseInitializerNode : public rclcpp::Node
{
public:
  explicit PoseInitializerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Client<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr
    get_trajectory_states_client_;
  rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_client_;
  rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_client_;
  void initialPoseCallback(const PoseWithCovarianceStamped::SharedPtr msg);

  template<typename T>
  void waitForService(const typename rclcpp::Client<T>::SharedPtr client);

  std::string configuration_directory_;
  std::string configuration_basename_;
};

}  // namespace pose_initializer
}  // namespace cartographer_utils

#endif  // POSE_INITIALIZER__POSE_INITIALIZER_NODE_HPP_

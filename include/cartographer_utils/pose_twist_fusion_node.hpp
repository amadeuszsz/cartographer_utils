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

#ifndef CARTOGRAPHER_UTILS__POSE_TWIST_FUSION_NODE_HPP_
#define CARTOGRAPHER_UTILS__POSE_TWIST_FUSION_NODE_HPP_

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "cartographer_utils/visibility_control.hpp"

namespace cartographer_utils
{
namespace pose_twist_fusion
{
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;

class CARTOGRAPHER_UTILS_PUBLIC PoseTwistFusionNode : public rclcpp::Node
{
public:
  explicit PoseTwistFusionNode(const rclcpp::NodeOptions & options);
  void poseCallback(const PoseStamped::SharedPtr msg);
  void poseWithCovCallback(const PoseWithCovarianceStamped::SharedPtr msg);
  void twistCallback(const TwistStamped::SharedPtr msg);
  void twistWithCovCallback(const TwistWithCovarianceStamped::SharedPtr msg);
  void timerOdometryCallback();

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr sub_twist_with_cov_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  Odometry odom_msg_;

  bool overwrite_pose_cov_;
  bool overwrite_twist_cov_;
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;
};
}  // namespace pose_twist_fusion
}  // namespace cartographer_utils

#endif  // CARTOGRAPHER_UTILS__POSE_TWIST_FUSION_NODE_HPP_

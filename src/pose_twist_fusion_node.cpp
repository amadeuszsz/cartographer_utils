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

#include "cartographer_utils/pose_twist_fusion_node.hpp"

namespace cartographer_utils
{
namespace pose_twist_fusion
{

PoseTwistFusionNode::PoseTwistFusionNode(const rclcpp::NodeOptions & options)
:  Node("pose_twist_fusion", options)
{
  auto odom_rate = this->declare_parameter<double>("odom_rate", 50.0);
  overwrite_pose_cov_ = this->declare_parameter<bool>("pose.overwrite_cov", false);
  overwrite_twist_cov_ = this->declare_parameter<bool>("twist.overwrite_cov", false);
  std::vector<double> pose_covariance =
    this->declare_parameter<std::vector<double>>("pose.cov");
  for (std::size_t i = 0; i < pose_covariance.size(); ++i) {
    pose_covariance_[i] = pose_covariance[i];
  }
  std::vector<double> twist_covariance =
    this->declare_parameter<std::vector<double>>("twist.cov");
  for (std::size_t i = 0; i < twist_covariance.size(); ++i) {
    twist_covariance_[i] = twist_covariance[i];
  }

  // Initialize base odom msg
  odom_msg_.header.frame_id = this->declare_parameter<std::string>("frame_id", "map");
  odom_msg_.child_frame_id = this->declare_parameter<std::string>("child_frame_id", "base_link");
  odom_msg_.pose.covariance = pose_covariance_;
  odom_msg_.twist.covariance = twist_covariance_;

  if (this->declare_parameter<bool>("pose.is_with_cov", true)) {
    sub_pose_with_cov_ = this->create_subscription<PoseWithCovarianceStamped>(
      "pose_with_covariance", rclcpp::QoS(1),
      std::bind(&PoseTwistFusionNode::poseWithCovCallback, this, std::placeholders::_1));
  } else {
    sub_pose_ = this->create_subscription<PoseStamped>(
      "pose", rclcpp::QoS(1),
      std::bind(&PoseTwistFusionNode::poseCallback, this, std::placeholders::_1));
  }

  if (this->declare_parameter<bool>("twist.is_with_cov", true)) {
    sub_twist_with_cov_ = this->create_subscription<TwistWithCovarianceStamped>(
      "twist_with_covariance", rclcpp::QoS(1),
      std::bind(&PoseTwistFusionNode::twistWithCovCallback, this, std::placeholders::_1));
  } else {
    sub_twist_ = this->create_subscription<TwistStamped>(
      "twist", rclcpp::QoS(1),
      std::bind(&PoseTwistFusionNode::twistCallback, this, std::placeholders::_1));
  }

  pub_odom_ = this->create_publisher<Odometry>("odometry", rclcpp::QoS(1));

  timer_ = this->create_wall_timer(
    rclcpp::Rate(odom_rate).period(),
    std::bind(&PoseTwistFusionNode::timerOdometryCallback, this));
}

void PoseTwistFusionNode::poseCallback(const PoseStamped::SharedPtr msg)
{
  odom_msg_.pose.pose = msg->pose;
}

void PoseTwistFusionNode::poseWithCovCallback(const PoseWithCovarianceStamped::SharedPtr msg)
{
  odom_msg_.pose.pose = msg->pose.pose;
  if (!overwrite_pose_cov_) {
    odom_msg_.pose.covariance = msg->pose.covariance;
  }
}

void PoseTwistFusionNode::twistCallback(const TwistStamped::SharedPtr msg)
{
  odom_msg_.twist.twist = msg->twist;
}

void PoseTwistFusionNode::twistWithCovCallback(const TwistWithCovarianceStamped::SharedPtr msg)
{
  odom_msg_.twist.twist = msg->twist.twist;
  if (!overwrite_twist_cov_) {
    odom_msg_.twist.covariance = msg->twist.covariance;
  }
}

void PoseTwistFusionNode::timerOdometryCallback()
{
  odom_msg_.header.stamp = this->now();
  pub_odom_->publish(odom_msg_);
}

}  // namespace pose_twist_fusion
}  // namespace cartographer_utils

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cartographer_utils::pose_twist_fusion::PoseTwistFusionNode)

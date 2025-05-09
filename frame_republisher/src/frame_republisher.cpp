// Copyright 2025 Jackson Huang
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

#include "frame_republisher/frame_republisher.hpp"

using namespace std::chrono_literals;

namespace frame_republisher
{

FrameRepublisher::FrameRepublisher()
: Node("frame_republisher")
{
  // Declare and get frame parameters
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "");
  this->declare_parameter<std::string>("base_frame", "");

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Publishers
  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "registered_scan", rclcpp::QoS(10));
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "lidar_odometry", rclcpp::QoS(10));

  // Subscriptions: republish to match upstream rates
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, rclcpp::QoS(10),
    std::bind(&FrameRepublisher::odomCallback, this, std::placeholders::_1));

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, rclcpp::QoS(10),
    std::bind(&FrameRepublisher::pointCloudCallback, this, std::placeholders::_1));
}

void FrameRepublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto out = *msg;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;
  pub_odom_->publish(out);

  geometry_msgs::msg::TransformStamped t;
  t.header = out.header;
  t.child_frame_id = out.child_frame_id;
  t.transform.translation.x = out.pose.pose.position.x;
  t.transform.translation.y = out.pose.pose.position.y;
  t.transform.translation.z = out.pose.pose.position.z;
  t.transform.rotation = out.pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

void FrameRepublisher::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto out = *msg;
  out.header.frame_id = odom_frame_;
  pub_cloud_->publish(out);
}

} // namespace frame_republisher

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frame_republisher::FrameRepublisher>());
  rclcpp::shutdown();
  return 0;
}
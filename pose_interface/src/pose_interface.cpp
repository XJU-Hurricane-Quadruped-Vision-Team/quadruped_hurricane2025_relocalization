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

#include "pose_interface/pose_interface.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

namespace pose_interface
{

    PoseInterface::PoseInterface()
        : Node("pose_interface")
    {
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("target_frame", "base_link");

        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("target_frame", target_frame_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("quadruped_pose", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&PoseInterface::onTimer, this));
    }

    void PoseInterface::onTimer()
    {
        try
        {
            auto tf_stamped = tf_buffer_->lookupTransform(
                map_frame_, target_frame_, tf2::TimePointZero);

            double x = tf_stamped.transform.translation.x;
            double y = tf_stamped.transform.translation.y;
            double z = tf_stamped.transform.translation.z;

            RCLCPP_INFO(this->get_logger(),
                        "[Global pose] : x=%.3f y=%.3f z=%.3f",
                        x, y, z);

            nav_msgs::msg::Odometry pose_msg;
            pose_msg.header.stamp = tf_stamped.header.stamp;
            pose_msg.header.frame_id = map_frame_;
            pose_msg.child_frame_id = target_frame_;

            pose_msg.pose.pose.position.x = x;
            pose_msg.pose.pose.position.y = y;
            pose_msg.pose.pose.position.z = z;
            pose_msg.pose.pose.orientation = tf_stamped.transform.rotation;

            pose_pub_->publish(pose_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Could not transform '%s' to '%s': %s",
                map_frame_.c_str(), target_frame_.c_str(), ex.what());
        }
    }

} // namespace pose_interface

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pose_interface = std::make_shared<pose_interface::PoseInterface>();
    rclcpp::spin(pose_interface);
    rclcpp::shutdown();
    return 0;
}
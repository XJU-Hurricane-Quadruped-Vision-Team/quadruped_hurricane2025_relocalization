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

#ifndef POSE_INTERFACE__POSE_INTERFACE_HPP_
#define POSE_INTERFACE__POSE_INTERFACE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace pose_interface
{

    class PoseInterface : public rclcpp::Node
    {
    public:
        PoseInterface();

    private:
        void onTimer();

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::string map_frame_;
        std::string target_frame_;
    };

} // namespace pose_interface

#endif // POSE_INTERFACE__POSE_INTERFACE_HPP_
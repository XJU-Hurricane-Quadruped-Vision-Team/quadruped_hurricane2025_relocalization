# Copyright 2025 Jackson Huang
# Copyright 2025 Jiale Jian
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#！/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

import math
from serial import Serial
from quadruped_task_manager.utils import quadruped_task_base

class QuadrupedTaskManager(Node):
    def __init__(self):
        super().__init__('quadruped_task_manager')
        self.get_logger().info('quadruped_task_manager node started')

        self.serial = Serial('/dev/ttyUSB0', 115200, timeout=1)

        # subscribe to quadruped_pose topic
        self.quadruped_pose_sub = self.create_subscription(
            Odometry,
            'quadruped_pose',
            self.quadruped_pose_callback,
            20)
        
        # declare parameters
        self.declare_parameter('x1', 0.0)
        self.declare_parameter('y1', 0.0)
        self.declare_parameter('x2', 1.0)
        self.declare_parameter('y2', 1.0)

        # get parameters
        self.x1 = self.get_parameter('x1').value
        self.y1 = self.get_parameter('y1').value
        self.x2 = self.get_parameter('x2').value
        self.y2 = self.get_parameter('y2').value
  

    def quadruped_pose_callback(self, pose_msg):
        
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        z = pose_msg.pose.pose.position.z

        q = [
            pose_msg.pose.pose.orientation.x, 
            pose_msg.pose.pose.orientation.y, 
            pose_msg.pose.pose.orientation.z, 
            pose_msg.pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(q)

        # convert to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        self.serial.write(f'{x},{y},{z},{roll},{pitch},{yaw}\n'.encode())


def main():
    rclpy.init()
    quadruped_task_manager = QuadrupedTaskManager()
    rclpy.spin(quadruped_task_manager)
    quadruped_task_manager.destroy_node()
    if rclpy.ok():
        quadruped_task_manager.get_logger().info('quadruped_task_manager node stopped')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
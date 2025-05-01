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

import math
import time

class QuadrupedTaskBase(Node):
    def __init__(self):
        super().__init__('quadruped_task_base')

    # 小马直走
    def quadruped_walk_straight(self, serial, x_p, y_p, differ_staight, k, b):
        self.get_logger().info(f"世界坐标:x=({x_p}),y=({y_p})\n")
        y = (y_p - k * x_p - b) / math.sqrt(k ** 2 + 1)

        if y > differ_staight :
            self.get_logger().info(f"边走边右转！！！\n")
            serial.write("s000010000000000000e\r\n".encode())

        elif y < -differ_staight :
            self.get_logger().info(f"边走边左转！！！\n")
            serial.write("s000000000000100000e\r\n".encode())

        else :
            self.get_logger().info(f"直直直走！！！\n")
            serial.write("s000000000001000000e\r\n".encode())

    # 小马竞速
    def quadruped_racing_process(self, serial, x_p, y_p, k, b):
        # 设置偏移直线宽度范围信息(纠正小小马偏离直线x轴投影的距离)(单位 m)
        differ_straight = 0.08

        # 设置欧拉角角度范围信息(纠正小小马偏离正前方朝向的角度)(单位 °)
        differ_yaw = 15

        self.quadruped_walk_straight(serial, x_p, y_p, differ_straight, k, b)

    def quadruped_axis_calculate(self, x1, y1, x2, y2):
        k = (y1 - y2) / (x1 - x2)
        b = y1 - k * x1
        
        return k, b
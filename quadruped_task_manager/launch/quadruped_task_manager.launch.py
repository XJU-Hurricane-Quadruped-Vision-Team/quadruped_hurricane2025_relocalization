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

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_task_manager',
            executable='quadruped_task_manager',
            name='quadruped_task_manager',
            output='screen',
            parameters=[
                {
                    'x1': 0.0,
                    'y1': 0.0,
                    'x2': 1.0,
                    'y2': 1.0,
                }
            ]
        )
    ])

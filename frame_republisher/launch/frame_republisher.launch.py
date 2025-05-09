# Copyright 2025 Jackson Huang
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
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    node = Node(
        package='frame_republisher',
        executable='frame_republisher_node',
        name='frame_republisher',
        output='screen',
        parameters=[
            {
                'state_estimation_topic': 'aft_mapped_to_init',
                'registered_scan_topic': 'cloud_registered',
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }
        ],
    )

    return LaunchDescription([node])

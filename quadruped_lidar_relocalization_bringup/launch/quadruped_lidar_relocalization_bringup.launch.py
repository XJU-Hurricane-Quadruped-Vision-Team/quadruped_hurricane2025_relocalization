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

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # get the launch directory
    bringup_dir = get_package_share_directory('quadruped_lidar_relocalization_bringup')

    remappings = []

    # Create the launch configuration variables    
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    lio = LaunchConfiguration("lio")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")


    # Declare the launch arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    declare_lio_cmd = DeclareLaunchArgument(
        "lio",
        default_value="fast_lio",
        description="Whether to use point_lio or fast_lio",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=os.path.join(
            bringup_dir, "pcd", "work_station.pcd"
        ),
        description="Full path to prior pcd file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "quadruped_lidar_relocalization_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "quadruped_lidar_relocalization.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RVIZ",
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True
        ),
        allow_substs=True
    )

    # Nodes to start
    livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ("/imu/data_raw", "/livox/imu")
        ]
    )

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        namespace=namespace,
        output="screen",
        remappings=remappings,
        parameters=[
            params_file,
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(lio == "point_lio")
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=[configured_params],
        output='screen',
        condition=IfCondition(lio == "fast_lio")
    )

    frame_republisher_node = Node(
        package='frame_republisher',
        executable='frame_republisher_node',
        name='frame_republisher',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
    )

    small_gicp_relocalization_node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace=namespace,
        output="screen",
        remappings=remappings,
        parameters=[
            configured_params,
            {"prior_pcd_file": prior_pcd_file}
        ],
    )

    pose_interface_node = Node(
        package='pose_interface',
        executable='pose_interface',
        name='pose_interface',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
    )

    quadruped_task_manager_node = Node(
        package='quadruped_task_manager',
        executable='quadruped_task_manager',
        name='quadruped_task_manager',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
    )


    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_lio_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(livox_ros_driver2_node)
    ld.add_action(imu_complementary_filter_node)
    ld.add_action(point_lio_node)
    ld.add_action(fast_lio_node)
    ld.add_action(frame_republisher_node)
    ld.add_action(small_gicp_relocalization_node)
    ld.add_action(pose_interface_node)
    ld.add_action(quadruped_task_manager_node)
    ld.add_action(rviz_cmd)

    return ld


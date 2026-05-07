#!/usr/bin/env python3
#
# Copyright 2022 ROBOTIS CO., LTD.
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
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    floor = LaunchConfiguration('floor')
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    topology_map_yaml_file = LaunchConfiguration('topology_map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    nav2_share = get_package_share_directory('robotics_nav2')
    warehouse_map_yaml_file = os.path.join(nav2_share, 'map', 'warehouse_map.yaml')
    office_map_yaml_file = os.path.join(nav2_share, 'map', 'office_map.yaml')
    warehouse_topology_map_yaml_file = os.path.join(nav2_share, 'map', 'warehouse_topology.yaml')
    office_topology_map_yaml_file = os.path.join(nav2_share, 'map', 'office_topology.yaml')
    warehouse_params_file = os.path.join(nav2_share, 'param', 'warehouse_nav2.yaml')
    office_params_file = os.path.join(nav2_share, 'param', 'office_nav2.yaml')
    warehouse_rviz_config_file = os.path.join(nav2_share, 'rviz', 'warehouse_nav2_lite.rviz')
    office_rviz_config_file = os.path.join(nav2_share, 'rviz', 'office_nav2_lite.rviz')

    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bringup'),
            'launch',
        ]
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    default_bt_xml_filename = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scene',
            default_value='warehouse',
            description='Indoor delivery scene profile: warehouse or office'),

        DeclareLaunchArgument(
            'floor',
            default_value='1f',
            description='Initial logical floor for single-floor delivery scenes'),

        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation'),

        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PythonExpression([
                "'", office_map_yaml_file, "' if '", scene, "' == 'office' else '",
                warehouse_map_yaml_file, "'"
            ]),
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'topology_map_yaml_file',
            default_value=PythonExpression([
                "'", office_topology_map_yaml_file, "' if '", scene, "' == 'office' else '",
                warehouse_topology_map_yaml_file, "'"
            ]),
            description='Full path to topology map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PythonExpression([
                "'", office_params_file, "' if '", scene, "' == 'office' else '",
                warehouse_params_file, "'"
            ]),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PythonExpression([
                "'", office_rviz_config_file, "' if '", scene, "' == 'office' else '",
                warehouse_rviz_config_file, "'"
            ]),
            description='Full path to the RViz config file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt_xml_filename,
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Whether to use composed bringup'),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Whether to respawn if a node crashes. \
                Applied when composition is disabled.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'topology_map': topology_map_yaml_file,
                'use_sim_time': use_sim,
                'params_file': params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)),
    ])

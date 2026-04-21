#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_composition = LaunchConfiguration('use_composition')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    scenario_start_delay = LaunchConfiguration('scenario_start_delay')

    description_dir = get_package_share_directory('robotics_description')
    nav_dir = get_package_share_directory('robotics_nav2')
    scenario_dir = get_package_share_directory('robotics_scenario')

    world_path = PythonExpression([
        "'", os.path.join(get_package_share_directory('robotics_gazebo'), 'worlds', 'hotel_delivery.world'),
        "' if '", scene, "' == 'hotel' else '",
        os.path.join(get_package_share_directory('robotics_gazebo'), 'worlds', 'warehouse_delivery.world'), "'"
    ])
    spawn_x = PythonExpression(["'3.5' if '", scene, "' == 'hotel' else '3.5'"])
    spawn_y = PythonExpression(["'2.0' if '", scene, "' == 'hotel' else '2.0'"])

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='hotel', description='Scene profile to launch'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start rviz'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Start gazebo GUI'),
        DeclareLaunchArgument('use_composition', default_value='False',
                              description='Use composed Nav2 bringup'),
        DeclareLaunchArgument('nav2_start_delay', default_value='30.0',
                              description='Seconds to wait for Gazebo spawn before starting Nav2'),
        DeclareLaunchArgument('scenario_start_delay', default_value='75.0',
                              description='Seconds to wait for Nav2 before starting scenario manager'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description_dir, 'launch', 'robotics_gazebo.launch.py')),
            launch_arguments={
                'start_rviz': use_rviz,
                'use_gazebo_gui': use_gazebo_gui,
                'world': world_path,
                'spawn_x': spawn_x,
                'spawn_y': spawn_y,
            }.items(),
        ),
        TimerAction(
            period=nav2_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'nav2_gazebo.launch.py')),
                    launch_arguments={
                        'scene': scene,
                        'start_rviz': use_rviz,
                        'use_composition': use_composition,
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=scenario_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(scenario_dir, 'launch', 'robotics_scenario.launch.py')),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                )
            ],
        ),
    ])

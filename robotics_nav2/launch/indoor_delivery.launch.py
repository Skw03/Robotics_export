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
    force_software_rendering = LaunchConfiguration('force_software_rendering')
    use_composition = LaunchConfiguration('use_composition')
    floor = LaunchConfiguration('floor')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    scenario_start_delay = LaunchConfiguration('scenario_start_delay')

    description_dir = get_package_share_directory('robotics_description')
    nav_dir = get_package_share_directory('robotics_nav2')
    scenario_dir = get_package_share_directory('robotics_scenario')
    warehouse_params_file = os.path.join(nav_dir, 'param', 'warehouse_nav2.yaml')
    office_params_file = os.path.join(nav_dir, 'param', 'office_nav2.yaml')

    world_path = PythonExpression([
        "'", os.path.join(get_package_share_directory('robotics_gazebo'), 'worlds', 'office.world'),
        "' if '", scene, "' == 'office' else '",
        os.path.join(get_package_share_directory('robotics_gazebo'), 'worlds', 'warehouse.world'), "'"
    ])
    spawn_x = PythonExpression(["'55.074' if '", scene, "' == 'office' else '-3.071'"])
    spawn_y = PythonExpression(["'-58.483' if '", scene, "' == 'office' else '3.583'"])

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='warehouse', description='Scene profile to launch'),
        DeclareLaunchArgument('floor', default_value='1f', description='Initial logical floor'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Start rviz'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='false', description='Start gazebo GUI'),
        DeclareLaunchArgument(
            'force_software_rendering',
            default_value='false',
            description='Use CPU software rendering for Gazebo when GPU rendering is unstable'),
        DeclareLaunchArgument('use_composition', default_value='False',
                              description='Use composed Nav2 bringup'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PythonExpression([
                "'", office_params_file, "' if '", scene, "' == 'office' else '",
                warehouse_params_file, "'"
            ]),
            description='Nav2 params file; pass a generated profile for planner/avoidance experiments'),
        DeclareLaunchArgument('nav2_start_delay', default_value='30.0',
                              description='Seconds to wait for Gazebo spawn before starting Nav2'),
        DeclareLaunchArgument('scenario_start_delay', default_value='75.0',
                              description='Seconds to wait for Nav2 before starting scenario manager'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description_dir, 'launch', 'robotics_gazebo.launch.py')),
            launch_arguments={
                'start_rviz': use_rviz,
                'use_gazebo_gui': use_gazebo_gui,
                'force_software_rendering': force_software_rendering,
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
                        'floor': floor,
                        'start_rviz': use_rviz,
                        'use_composition': use_composition,
                        'params_file': nav2_params_file,
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

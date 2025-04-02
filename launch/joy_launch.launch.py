#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    turtlebot3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                         'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'map': os.path.join(get_package_share_directory('patrolling_bot'), 'map', 'my_map.yaml')}.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    joy_cmd_vel_package_cpp = Node(
        package='patrolling_bot',
        executable='joy_service_node',
        output='screen'
    )

    return LaunchDescription([
        turtlebot3_model_env,
        navigation_launch,
        joy_node,
        joy_cmd_vel_package_cpp
    ])
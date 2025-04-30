#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtlebot3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    turtlebot3_house_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'launch', 'turtlebot3_dqn_stage4.launch.py')
        )
    )
    return LaunchDescription([
        turtlebot3_model_env,
        turtlebot3_house_launch,
    ])

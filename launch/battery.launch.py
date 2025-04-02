#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

   
 

    bot_curr_battery_node = Node(
        package='patrolling_bot',
        executable='bot_curr_battery_node',
        output='screen'
    )

    return LaunchDescription([
    
        bot_curr_battery_node
    ])
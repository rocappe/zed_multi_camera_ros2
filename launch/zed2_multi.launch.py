#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    zed_l_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed2_l.launch.py'])
        )

    zed_r_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed2_r.launch.py'])
        )


    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch parameters
    ld.add_action(zed_l_launch)

    # Add nodes to LaunchDescription
    ld.add_action(TimerAction(period=20., actions=[zed_r_launch]))

    return ld

#!usr/bin/env python 3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mango',
            executable='webcam_driver',
            name='webcam_driver',
            output='screen'
        ),
        Node(
            package='blue_filter',
            executable='color_filter',
            name='color_filter',
            output='screen'
        )
    ])

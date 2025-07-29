#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='robot',
        executable='motor_controller_node', # Nazwa z setup.py
        name='motor_controller',
        output='screen'
    )

    ])

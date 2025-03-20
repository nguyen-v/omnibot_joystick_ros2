#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joy_to_cmd_vel_node = Node(
        package='joy_to_cmd_vel',  # This is your package name
        executable='joy_to_cmd_vel_node',  # Must match the entry point name defined in setup.py
        name='joy_to_cmd_vel_node',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        joy_to_cmd_vel_node,
    ])

#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='main_pkg',
            executable='spawn_cube.py',
            output='screen',
            arguments=[
                '--robot_urdf', launch.substitutions.LaunchConfiguration('object_urdf'),
                '--robot_name', launch.substitutions.LaunchConfiguration('object_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('object_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z'),
                '-qx', launch.substitutions.LaunchConfiguration('qx'),
                '-qy', launch.substitutions.LaunchConfiguration('qy'),
                '-qz', launch.substitutions.LaunchConfiguration('qz'),
                '-qw', launch.substitutions.LaunchConfiguration('qw')]),
    ])

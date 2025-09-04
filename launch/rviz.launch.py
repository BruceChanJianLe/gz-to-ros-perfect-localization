#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # Include map to world launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gz-to-ros-perfect-localization"),
                                "launch",
                                "map_to_world.launch.py",
                            ]
                        )
                    ]
                ),
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="perf_loc_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("gz-to-ros-perfect-localization"),
                            "rviz",
                            "config.rviz",
                        ]
                    ),
                ],
            ),
        ]
    )

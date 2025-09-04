#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            # Map to World Static Transform Arguments (map -> world)
            DeclareLaunchArgument(
                "map_to_world_x",
                default_value="0.0",
                description="Map to world X translation",
            ),
            DeclareLaunchArgument(
                "map_to_world_y",
                default_value="0.0",
                description="Map to world Y translation",
            ),
            DeclareLaunchArgument(
                "map_to_world_z",
                default_value="0.0",
                description="Map to world Z translation",
            ),
            DeclareLaunchArgument(
                "map_to_world_roll",
                default_value="0.0",
                description="Map to world roll rotation (radians)",
            ),
            DeclareLaunchArgument(
                "map_to_world_pitch",
                default_value="0.0",
                description="Map to world pitch rotation (radians)",
            ),
            DeclareLaunchArgument(
                "map_to_world_yaw",
                default_value="0.0",
                description="Map to world yaw rotation (radians)",
            ),

            # Static transform publisher (map -> world)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_world_static_tf",
                output="screen",
                arguments=[
                    LaunchConfiguration("map_to_world_x"),
                    LaunchConfiguration("map_to_world_y"),
                    LaunchConfiguration("map_to_world_z"),
                    LaunchConfiguration("map_to_world_roll"),
                    LaunchConfiguration("map_to_world_pitch"),
                    LaunchConfiguration("map_to_world_yaw"),
                    "map",
                    "world",
                ],
            ),
        ]
    )

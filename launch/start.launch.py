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
            # Declare launch arguments
            DeclareLaunchArgument(
                "gz_topic",
                default_value="/world/empty_world/pose/info",
                description="Gazebo pose topic to subscribe to",
            ),
            DeclareLaunchArgument(
                "target_entity_name",
                default_value="robot",
                description="Name of the entity to track",
            ),
            DeclareLaunchArgument(
                "parent_frame",
                default_value="world",
                description="Parent frame for TF transform",
            ),
            DeclareLaunchArgument(
                "amcl_parent_frame",
                default_value="map",
                description="AMCL Parent frame for TF transform",
            ),
            DeclareLaunchArgument(
                "child_frame",
                default_value="base_link",
                description="Child frame for TF transform",
            ),
            DeclareLaunchArgument(
                "publish_amcl_pose",
                default_value="true",
                description="Whether to publish fake AMCL pose",
            ),
            DeclareLaunchArgument(
                "amcl_topic",
                default_value="/amcl_pose",
                description="Topic to publish AMCL pose on",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Whether to publish TF transforms",
            ),

            # Static transform arguments for frame correction
            DeclareLaunchArgument(
                "static_transform_x",
                default_value="0.0",
                description="Static transform X offset",
            ),
            DeclareLaunchArgument(
                "static_transform_y",
                default_value="0.0",
                description="Static transform Y offset",
            ),
            DeclareLaunchArgument(
                "static_transform_z",
                default_value="0.0",
                description="Static transform Z offset",
            ),
            DeclareLaunchArgument(
                "static_transform_roll",
                default_value="0.0",
                description="Static transform roll (radians)",
            ),
            DeclareLaunchArgument(
                "static_transform_pitch",
                default_value="0.0",
                description="Static transform pitch (radians)",
            ),
            DeclareLaunchArgument(
                "static_transform_yaw",
                default_value="0.0",
                description="Static transform yaw (radians)",
            ),

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

            # Launch the bridge node
            Node(
                package="gz-to-ros-perfect-localization",
                executable="perfect_localization",
                name="perfect_localization_node",
                output="screen",
                parameters=[
                    {
                        "gz_topic": LaunchConfiguration("gz_topic"),
                        "target_entity_name": LaunchConfiguration("target_entity_name"),
                        "parent_frame": LaunchConfiguration("parent_frame"),
                        "child_frame": LaunchConfiguration("child_frame"),
                        "publish_amcl_pose": LaunchConfiguration("publish_amcl_pose"),
                        "amcl_topic": LaunchConfiguration("amcl_topic"),
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "static_transform_x": LaunchConfiguration("static_transform_x"),
                        "static_transform_y": LaunchConfiguration("static_transform_y"),
                        "static_transform_z": LaunchConfiguration("static_transform_z"),
                        "static_transform_roll": LaunchConfiguration(
                            "static_transform_roll"
                        ),
                        "static_transform_pitch": LaunchConfiguration(
                            "static_transform_pitch"
                        ),
                        "static_transform_yaw": LaunchConfiguration(
                            "static_transform_yaw"
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )

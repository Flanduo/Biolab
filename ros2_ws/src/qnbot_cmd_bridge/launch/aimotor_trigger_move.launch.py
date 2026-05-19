#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("qnbot_cmd_bridge")
    config_file = os.path.join(pkg_share, "config", "aimotor_trigger_move.yaml")

    # If running from a workspace with src available, prefer src config.
    parts = os.path.normpath(pkg_share).split(os.sep)
    if "install" in parts:
        idx = parts.index("install")
        src_base = os.sep.join(parts[:idx] + ["src", "qnbot_cmd_bridge"])
        src_config = os.path.join(src_base, "config", "aimotor_trigger_move.yaml")
        if os.path.isfile(src_config):
            config_file = src_config

    use_src = DeclareLaunchArgument(
        "use_src_config",
        default_value="false",
        description="Reserved compatibility argument",
    )

    target_pose_file = DeclareLaunchArgument(
        "target_pose_file",
        default_value="1.md",
        description="JointState text file path used to load target joint positions",
    )
    trigger_threshold = DeclareLaunchArgument(
        "trigger_threshold",
        default_value="0.2",
        description="Trigger when state value is smaller than this threshold",
    )
    release_threshold = DeclareLaunchArgument(
        "release_threshold",
        default_value="0.25",
        description="Re-arm only when state value is larger than this threshold",
    )

    node = Node(
        package="qnbot_cmd_bridge",
        executable="aimotor_trigger_move_node",
        name="aimotor_trigger_move_node",
        output="screen",
        parameters=[
            config_file,
            {
                "target_pose_file": LaunchConfiguration("target_pose_file"),
                "trigger_threshold": LaunchConfiguration("trigger_threshold"),
                "release_threshold": LaunchConfiguration("release_threshold"),
            },
        ],
    )

    return LaunchDescription(
        [
            use_src,
            target_pose_file,
            trigger_threshold,
            release_threshold,
            node,
        ]
    )

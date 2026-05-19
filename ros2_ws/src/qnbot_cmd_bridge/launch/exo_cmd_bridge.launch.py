#!/usr/bin/env python3
"""
启动外骨骼命令桥接节点。
需先运行 qnbot_teleoperator 的 websocket_teleoperator，确保 /exo/gamepad_keys 有数据。

用法:
  ros2 launch qnbot_cmd_bridge exo_cmd_bridge.launch.py
  ros2 launch qnbot_cmd_bridge exo_cmd_bridge.launch.py use_src_config:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("qnbot_cmd_bridge")
    config_file = os.path.join(pkg_share, "config", "exo_cmd_bridge.yaml")
    # 若在源码目录运行，优先使用源码 config
    parts = os.path.normpath(pkg_share).split(os.sep)
    if "install" in parts:
        idx = parts.index("install")
        src_base = os.sep.join(parts[:idx] + ["src", "qnbot_cmd_bridge"])
        src_config = os.path.join(src_base, "config", "exo_cmd_bridge.yaml")
        if os.path.isfile(src_config):
            config_file = src_config

    use_src = DeclareLaunchArgument(
        "use_src_config",
        default_value="false",
        description="若为 true 且存在源码 config 则使用源码配置",
    )

    node = Node(
        package="qnbot_cmd_bridge",
        executable="exo_cmd_bridge_node",
        name="exo_cmd_bridge_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([use_src, node])

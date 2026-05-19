#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _resolve_workspace_path() -> str:
    """
    Resolve workspace root from installed package share path:
      .../<ws>/install/openarm_bringup/share/openarm_bringup
    """
    pkg_share = get_package_share_directory("openarm_bringup")
    parts = os.path.normpath(pkg_share).split(os.sep)
    if "install" in parts:
        idx = parts.index("install")
        return os.sep.join(parts[:idx])
    return os.path.expanduser("~/ros2_ws")


def generate_launch_description():
    pkg_share = get_package_share_directory("openarm_bringup")
    bimanual_launch = os.path.join(pkg_share, "launch", "openarm.bimanual.launch.py")

    ws_root = _resolve_workspace_path()
    script_path = os.path.join(
        ws_root, "src", "openarm_teleop", "script", "aimotor_trigger_move_bimanual.py"
    )
    default_config = os.path.join(
        ws_root, "src", "openarm_teleop", "config", "aimotor_trigger_move.yaml"
    )

    config_arg = DeclareLaunchArgument(
        "trigger_config",
        default_value=default_config,
        description="Config YAML for aimotor trigger node",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Whether to launch RViz from bringup",
    )

    include_openarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bimanual_launch),
        launch_arguments={
            "arm_type": "v10",
            "robot_controller": "forward_position_controller",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    trigger_process = ExecuteProcess(
        cmd=[
            "python3",
            script_path,
            "--config",
            LaunchConfiguration("trigger_config"),
        ],
        output="screen",
    )

    delayed_trigger = TimerAction(
        period=2.0,
        actions=[trigger_process],
    )

    return LaunchDescription(
        [
            config_arg,
            launch_rviz_arg,
            include_openarm,
            delayed_trigger,
        ]
    )

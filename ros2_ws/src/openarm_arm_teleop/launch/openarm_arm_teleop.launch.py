import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("openarm_arm_teleop")
    joint_config_file = os.path.join(pkg_share, "config", "openarm_arm_teleop.yaml")
    ee_config_file = os.path.join(pkg_share, "config", "openarm_ee_teleop.yaml")
    mode = LaunchConfiguration("mode")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="joint",
        description="Control mode: joint or ee",
    )

    joint_node = Node(
        package="openarm_arm_teleop",
        executable="arm_teleop_node",
        name="openarm_f710_teleop",
        output="screen",
        parameters=[joint_config_file],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joint'"])),
    )

    ee_node = Node(
        package="openarm_arm_teleop",
        executable="ee_teleop_node",
        name="openarm_ee_teleop",
        output="screen",
        parameters=[ee_config_file],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'ee'"])),
    )

    return LaunchDescription([mode_arg, joint_node, ee_node])

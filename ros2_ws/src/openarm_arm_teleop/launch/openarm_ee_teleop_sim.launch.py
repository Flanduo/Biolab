from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("openarm_bimanual_moveit_config"),
                    "launch",
                    "demo.launch.py",
                ]
            )
        ),
        launch_arguments={"arm_type": "v10", "use_fake_hardware": "true"}.items(),
    )

    ee_teleop_node = Node(
        package="openarm_arm_teleop",
        executable="ee_teleop_node",
        name="openarm_ee_teleop",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("openarm_arm_teleop"),
                    "config",
                    "openarm_ee_teleop.yaml",
                ]
            )
        ],
    )

    return LaunchDescription([moveit_demo, ee_teleop_node])

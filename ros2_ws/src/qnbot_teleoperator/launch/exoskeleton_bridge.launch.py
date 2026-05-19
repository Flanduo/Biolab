import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    gripper_threshold_arg = DeclareLaunchArgument(
        'gripper_threshold',
        default_value='0.005',
        description='夹爪位置变化的最小阈值 (m)，用于减少Action请求频率'
    )
    
    gripper_scaling_factor_arg = DeclareLaunchArgument(
        'gripper_scaling_factor',
        default_value='0.02',
        description='夹爪数值缩放因子：外骨骼归一化值(0-1) -> 机械臂物理值(米)。默认0.02表示外骨骼1.0对应机械臂2cm'
    )

    # 桥接节点
    bridge_node = Node(
        package='qnbot_teleoperator',
        executable='exoskeleton_bridge_node',
        name='exoskeleton_bridge_node',
        output='screen',
        parameters=[{
            'gripper_threshold': LaunchConfiguration('gripper_threshold'),
            'gripper_scaling_factor': LaunchConfiguration('gripper_scaling_factor')
        }]
    )

    return LaunchDescription([
        gripper_threshold_arg,
        gripper_scaling_factor_arg,
        bridge_node
    ])


"""
OpenArm SDK 的配置类

本模块提供了机械臂类型、控制模式和其他设置的配置类。
"""

from enum import Enum
from typing import List, Optional

try:
    from openarm.can import MotorType
except ImportError:
    # 如果 openarm_can 不可用的回退方案
    class MotorType:
        DM4310 = None


class ControlMode(Enum):
    """OpenArm 的控制模式"""
    
    DISABLED = "disabled"  # 禁用
    GRAVITY_COMPENSATION = "gravity_compensation"  # 重力补偿
    POSITION_CONTROL = "position_control"  # 位置控制
    VELOCITY_CONTROL = "velocity_control"  # 速度控制
    TORQUE_CONTROL = "torque_control"  # 力矩控制
    BILATERAL_CONTROL = "bilateral_control"  # 双边控制


class ArmConfig:
    """
    OpenArm 设置的配置类
    
    此类保存初始化机械臂的配置参数，
    包括电机类型、CAN ID 和其他设置。
    """
    
    # 不同机械臂类型的默认配置
    V10_CONFIG = {
        "motor_types": [MotorType.DM4310] * 7,  # 7 个机械臂电机
        "send_can_ids": [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
        "recv_can_ids": [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
        "gripper_motor_type": MotorType.DM4310,
        "gripper_send_can_id": 0x08,
        "gripper_recv_can_id": 0x18,
    }
    
    def __init__(
        self,
        arm_type: str = "v10",
        motor_types: Optional[List[MotorType]] = None,
        send_can_ids: Optional[List[int]] = None,
        recv_can_ids: Optional[List[int]] = None,
        gripper_motor_type: Optional[MotorType] = None,
        gripper_send_can_id: Optional[int] = None,
        gripper_recv_can_id: Optional[int] = None,
    ):
        """
        初始化机械臂配置
        
        参数:
            arm_type: 机械臂类型 (默认: "v10")
            motor_types: 机械臂关节的电机类型列表
            send_can_ids: 机械臂关节的发送 CAN ID 列表
            recv_can_ids: 机械臂关节的接收 CAN ID 列表
            gripper_motor_type: 夹爪的电机类型
            gripper_send_can_id: 夹爪的发送 CAN ID
            gripper_recv_can_id: 夹爪的接收 CAN ID
        """
        self.arm_type = arm_type
        
        # 如果未指定则使用默认配置
        if arm_type == "v10" and motor_types is None:
            default = self.V10_CONFIG
            motor_types = default["motor_types"]
            send_can_ids = send_can_ids or default["send_can_ids"]
            recv_can_ids = recv_can_ids or default["recv_can_ids"]
            gripper_motor_type = gripper_motor_type or default["gripper_motor_type"]
            gripper_send_can_id = gripper_send_can_id or default["gripper_send_can_id"]
            gripper_recv_can_id = gripper_recv_can_id or default["gripper_recv_can_id"]
        
        self.motor_types = motor_types or []
        self.send_can_ids = send_can_ids or []
        self.recv_can_ids = recv_can_ids or []
        self.gripper_motor_type = gripper_motor_type
        self.gripper_send_can_id = gripper_send_can_id
        self.gripper_recv_can_id = gripper_recv_can_id
        
        # 验证配置
        if len(self.motor_types) != len(self.send_can_ids) or \
           len(self.motor_types) != len(self.recv_can_ids):
            raise ValueError(
                "motor_types、send_can_ids 和 recv_can_ids 必须具有相同的长度"
            )


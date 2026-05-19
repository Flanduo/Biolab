"""
OpenArm 机器人控制的核心 SDK 类

本模块提供了主要的 OpenArmSDK 类，作为控制 OpenArm 机械臂的主要接口。
"""

import sys
import os
from typing import Optional, List, Dict, Any
from enum import Enum
import time

# 将 openarm_can Python 绑定添加到路径中（兼容多种目录布局）
_sdk_pkg_dir = os.path.dirname(os.path.abspath(__file__))  # .../openarm_sdk/openarm_sdk
_sdk_repo_dir = os.path.dirname(_sdk_pkg_dir)  # .../openarm_sdk
_workspace_dir = os.path.dirname(_sdk_repo_dir)  # .../ros2_ws

_OPENARM_CAN_PATH_CANDIDATES = [
    os.path.join(_workspace_dir, "src", "openarm_can", "python"),
    os.path.join(_sdk_repo_dir, "src", "openarm_can", "python"),
]

for _path in _OPENARM_CAN_PATH_CANDIDATES:
    if os.path.exists(_path) and _path not in sys.path:
        sys.path.insert(0, _path)

try:
    # 尝试导入 openarm_can Python 绑定
    import openarm.can as openarm_can
    from openarm.can import OpenArm, MotorType, CallbackMode
except ImportError:
    openarm_can = None
    OpenArm = None
    MotorType = None
    CallbackMode = None


from .config import ArmConfig, ControlMode
from .control import ControlLoop, GravityCompensationControl
from .dynamics import DynamicsInterface, SimpleDynamics
from .config_loader import ConfigLoader
from .control_params import ControlParameters
from .exceptions import (
    OpenArmSDKError,
    MotorError,
    ConnectionError,
    ConfigurationError,
    ControlError,
)


class OpenArmSDK:
    """
    OpenArm 机器人控制的主 SDK 类
    
    此类提供了控制 OpenArm 机械臂的高级接口，
    封装了底层 CAN 通信并为常见操作提供便捷方法。
    
    示例:
        >>> sdk = OpenArmSDK(can_interface="can0", enable_can_fd=True)
        >>> sdk.enable_all()
        >>> sdk.set_zero_all()
        >>> sdk.set_control_mode(ControlMode.GRAVITY_COMPENSATION)
        >>> sdk.start_control_loop()
    """
    
    def __init__(
        self,
        can_interface: str = "can0",
        enable_can_fd: bool = True,
        arm_type: str = "v10",
        config: Optional[ArmConfig] = None,
    ):
        """
        初始化 OpenArm SDK
        
        参数:
            can_interface: CAN 接口名称 (例如: "can0", "can1")
            enable_can_fd: 是否启用 CAN-FD 模式
            arm_type: 机械臂类型 (默认: "v10")
            config: 可选的 ArmConfig 对象，用于高级配置
        
        异常:
            ConnectionError: 如果无法打开 CAN 接口
            ImportError: 如果 openarm_can Python 绑定不可用
        """
        if openarm_can is None:
            raise ImportError(
                "openarm_can Python bindings not found. "
                "Please build the openarm_can library first."
            )
        
        self.can_interface = can_interface
        self.enable_can_fd = enable_can_fd
        self.arm_type = arm_type
        self.config = config or ArmConfig(arm_type=arm_type)
        
        # 初始化底层 OpenArm 对象
        try:
            self._openarm = OpenArm(can_interface, enable_can_fd)
        except Exception as e:
            raise ConnectionError(
                f"无法连接到 CAN 接口 {can_interface}: {e}"
            ) from e
        
        # 控制状态
        self._control_mode = None
        self._control_loop: Optional[ControlLoop] = None
        
    def init_arm_motors(
        self,
        motor_types: Optional[List[MotorType]] = None,
        send_can_ids: Optional[List[int]] = None,
        recv_can_ids: Optional[List[int]] = None,
    ):
        """
        初始化机械臂电机
        
        参数:
            motor_types: 电机类型列表 (默认: 使用配置)
            send_can_ids: 发送 CAN ID 列表 (默认: 使用配置)
            recv_can_ids: 接收 CAN ID 列表 (默认: 使用配置)
        """
        if motor_types is None:
            motor_types = self.config.motor_types
        if send_can_ids is None:
            send_can_ids = self.config.send_can_ids
        if recv_can_ids is None:
            recv_can_ids = self.config.recv_can_ids
        
        self._openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids)
    
    def init_gripper_motor(
        self,
        motor_type: Optional[MotorType] = None,
        send_can_id: Optional[int] = None,
        recv_can_id: Optional[int] = None,
    ):
        """
        初始化夹爪电机
        
        参数:
            motor_type: 电机类型 (默认: 使用配置)
            send_can_id: 发送 CAN ID (默认: 使用配置)
            recv_can_id: 接收 CAN ID (默认: 使用配置)
        """
        if motor_type is None:
            motor_type = self.config.gripper_motor_type
        if send_can_id is None:
            send_can_id = self.config.gripper_send_can_id
        if recv_can_id is None:
            recv_can_id = self.config.gripper_recv_can_id
        
        self._openarm.init_gripper_motor(motor_type, send_can_id, recv_can_id)
    
    def enable_all(self):
        """使能所有电机"""
        try:
            self._openarm.enable_all()
        except Exception as e:
            raise MotorError(f"无法使能电机: {e}") from e
    
    def disable_all(self):
        """禁用所有电机"""
        try:
            self._openarm.disable_all()
        except Exception as e:
            raise MotorError(f"无法禁用电机: {e}") from e
    
    def set_zero_all(self):
        """设置所有电机的零位"""
        try:
            self._openarm.set_zero_all()
        except Exception as e:
            raise MotorError(f"无法设置零位: {e}") from e
    
    def refresh_all(self):
        """刷新所有电机状态"""
        self._openarm.refresh_all()
    
    def recv_all(self, timeout_us: int = 500):
        """
        从所有电机接收数据
        
        参数:
            timeout_us: 超时时间（微秒）
        """
        self._openarm.recv_all(timeout_us)
    
    def get_arm(self):
        """获取机械臂组件"""
        return self._openarm.get_arm()
    
    def get_gripper(self):
        """获取夹爪组件"""
        return self._openarm.get_gripper()
    
    def set_control_mode(
        self, 
        mode: ControlMode,
        dynamics: Optional[DynamicsInterface] = None,
    ):
        """
        设置控制模式
        
        参数:
            mode: 控制模式 (例如: ControlMode.GRAVITY_COMPENSATION)
            dynamics: 动力学计算接口（可选，用于重力补偿模式）
        """
        # 如果已有控制循环在运行，先停止
        if self._control_loop is not None and self._control_loop.running:
            self.stop_control_loop()
        
        self._control_mode = mode
        
        # 根据模式创建相应的控制循环
        if mode == ControlMode.GRAVITY_COMPENSATION:
            self._control_loop = GravityCompensationControl(self, dynamics=dynamics)
        else:
            # 其他模式待实现
            raise ConfigurationError(f"控制模式 {mode} 尚未实现")
    
    def start_control_loop(self, frequency: float = 100.0):
        """
        启动控制循环
        
        参数:
            frequency: 控制频率（Hz），默认 100Hz
        """
        if self._control_mode is None:
            raise ConfigurationError("控制模式未设置。请先调用 set_control_mode()。")
        
        if self._control_loop is None:
            raise ConfigurationError("控制循环未初始化")
        
        if self._control_loop.running:
            print("警告: 控制循环已在运行")
            return
        
        self._control_loop.start()
    
    def stop_control_loop(self):
        """停止控制循环"""
        if self._control_loop is not None:
            self._control_loop.stop()
    
    def load_control_params(self, config_path: str, param_name: str) -> ControlParameters:
        """
        从配置文件加载控制参数
        
        参数:
            config_path: 配置文件路径
            param_name: 参数节点名称（例如 "LeaderArmParam"）
        
        返回:
            ControlParameters 实例
        """
        loader = ConfigLoader(config_path)
        return ControlParameters.from_config_loader(loader, param_name)
    
    def close(self):
        """关闭连接并清理资源"""
        if self._control_loop is not None and self._control_loop.running:
            self.stop_control_loop()
        # OpenArm 对象将自动清理
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.close()


"""
OpenArm SDK - OpenArm 机械臂控制的高级 Python SDK

本包提供了易于使用的接口来控制 OpenArm 机器人，
包括基本的电机控制、重力补偿、双边控制等功能。
"""

__version__ = "0.2.0"
__author__ = "OpenArm Team"

# 导入主要的 SDK 类
try:
    from .core import OpenArmSDK
except Exception as e:
    # 如果导入失败，提供更详细的错误信息
    import sys
    import traceback
    error_msg = (
        f"无法导入 OpenArmSDK: {e}\n"
        "可能原因：\n"
        "  1. openarm_can Python 绑定未构建或未安装\n"
        "  2. 依赖模块导入失败\n"
        "  3. core.py 模块内部错误\n"
        "\n解决方案：\n"
        "  cd ~/ros2_ws/src/openarm_can/python\n"
        "  pip3 install --no-build-isolation -e .\n"
        "\n详细错误信息：\n"
    )
    # 打印详细错误信息用于调试
    import traceback
    full_error = "".join(traceback.format_exception(type(e), e, e.__traceback__))
    raise ImportError(error_msg + full_error) from e

from .config import ArmConfig, ControlMode

# 导入控制相关类
from .control import ControlLoop, GravityCompensationControl
from .control_params import ControlParameters

# 导入配置加载器
from .config_loader import ConfigLoader

# 导入动力学接口
try:
    from .dynamics import DynamicsInterface, CallbackDynamics, SimpleDynamics, KDL_AVAILABLE
except ImportError:
    # KDL 可能不可用，这是可选的
    from .dynamics import DynamicsInterface, CallbackDynamics, SimpleDynamics
    KDL_AVAILABLE = False

# 导入异常类
from .exceptions import (
    OpenArmSDKError,
    MotorError,
    ConnectionError,
    ConfigurationError,
    ControlError,
)
from .exo import ExoProtocolParser

__all__ = [
    # 主要类
    "OpenArmSDK",
    "ArmConfig",
    "ControlMode",
    # 控制相关
    "ControlLoop",
    "GravityCompensationControl",
    "ControlParameters",
    "ConfigLoader",
    # 动力学接口
    "DynamicsInterface",
    "CallbackDynamics",
    "SimpleDynamics",
    "KDL_AVAILABLE",
    # 异常类
    "OpenArmSDKError",
    "MotorError",
    "ConnectionError",
    "ConfigurationError",
    "ControlError",
    # Exoskeleton SDK
    "ExoProtocolParser",
]


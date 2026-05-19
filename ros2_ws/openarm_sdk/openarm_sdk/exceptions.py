"""
OpenArm SDK 的异常类

本模块定义了在整个 SDK 中使用的自定义异常。
"""


class OpenArmSDKError(Exception):
    """所有 OpenArm SDK 错误的基础异常"""
    pass


class ConnectionError(OpenArmSDKError):
    """当连接到机器人失败时抛出"""
    pass


class MotorError(OpenArmSDKError):
    """当电机操作失败时抛出"""
    pass


class ConfigurationError(OpenArmSDKError):
    """当配置无效或缺失时抛出"""
    pass


class ControlError(OpenArmSDKError):
    """当控制操作失败时抛出"""
    pass


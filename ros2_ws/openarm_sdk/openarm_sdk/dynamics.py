"""
动力学计算接口

本模块提供了动力学计算的接口，支持通过回调函数或 Python KDL 绑定计算重力补偿。

由于不能修改 src 目录，本模块提供了灵活的接口，允许用户：
1. 使用 Python KDL 绑定（如果可用）
2. 使用回调函数注入自定义动力学计算
3. 使用简化模型（适用于快速原型）
"""

import os
from typing import Optional, Callable, List
from abc import ABC, abstractmethod

# 尝试导入 Python KDL 绑定（如果可用）
try:
    import PyKDL
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False
    PyKDL = None


class DynamicsInterface(ABC):
    """
    动力学计算接口基类
    
    定义动力学计算的抽象接口，子类可以实现具体的计算逻辑。
    """
    
    @abstractmethod
    def get_gravity(self, joint_positions: List[float]) -> List[float]:
        """
        计算重力扭矩
        
        参数:
            joint_positions: 关节位置列表（弧度）
        
        返回:
            重力扭矩列表（Nm）
        """
        pass
    
    @abstractmethod
    def get_coriolis(self, joint_positions: List[float], 
                     joint_velocities: List[float]) -> List[float]:
        """
        计算科里奥利力
        
        参数:
            joint_positions: 关节位置列表（弧度）
            joint_velocities: 关节速度列表（rad/s）
        
        返回:
            科里奥利力列表（Nm）
        """
        pass


class CallbackDynamics(DynamicsInterface):
    """
    基于回调函数的动力学计算
    
    允许用户通过回调函数注入动力学计算逻辑。
    适用于已有 C++ 动力学库绑定的情况。
    """
    
    def __init__(
        self,
        gravity_callback: Callable[[List[float]], List[float]],
        coriolis_callback: Optional[Callable[[List[float], List[float]], List[float]]] = None,
    ):
        """
        初始化回调动力学计算
        
        参数:
            gravity_callback: 重力计算回调函数
            coriolis_callback: 科里奥利力计算回调函数（可选）
        """
        self.gravity_callback = gravity_callback
        self.coriolis_callback = coriolis_callback or (lambda pos, vel: [0.0] * len(pos))
    
    def get_gravity(self, joint_positions: List[float]) -> List[float]:
        """计算重力扭矩"""
        return self.gravity_callback(joint_positions)
    
    def get_coriolis(self, joint_positions: List[float], 
                     joint_velocities: List[float]) -> List[float]:
        """计算科里奥利力"""
        return self.coriolis_callback(joint_positions, joint_velocities)


class SimpleDynamics(DynamicsInterface):
    """
    简化的动力学模型
    
    使用简化的模型进行重力补偿，适用于快速原型和测试。
    注意：这不是精确的动力学模型，仅用于演示。
    """
    
    def __init__(self, num_joints: int = 7):
        """
        初始化简化动力学模型
        
        参数:
            num_joints: 关节数量
        """
        self.num_joints = num_joints
        # 简化的重力系数（需要根据实际机械臂调整）
        self.gravity_coeffs = [0.5, 0.8, 0.6, 0.4, 0.3, 0.2, 0.1]
    
    def get_gravity(self, joint_positions: List[float]) -> List[float]:
        """
        计算重力扭矩（简化模型）
        
        使用简单的三角函数近似重力补偿。
        """
        gravity_torques = []
        for i, pos in enumerate(joint_positions[:self.num_joints]):
            # 简化的重力模型：g * sin(theta) * coeff
            import math
            torque = 9.81 * math.sin(pos) * self.gravity_coeffs[i]
            gravity_torques.append(torque)
        
        # 填充剩余关节（如果有）
        while len(gravity_torques) < len(joint_positions):
            gravity_torques.append(0.0)
        
        return gravity_torques
    
    def get_coriolis(self, joint_positions: List[float], 
                     joint_velocities: List[float]) -> List[float]:
        """计算科里奥利力（简化模型，返回零）"""
        return [0.0] * len(joint_positions)


# 导出接口
__all__ = [
    "DynamicsInterface",
    "CallbackDynamics",
    "SimpleDynamics",
    "KDL_AVAILABLE",
]


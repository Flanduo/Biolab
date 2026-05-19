"""
控制参数类

本模块定义了控制参数的数据结构，用于存储和传递控制参数。
"""

from typing import List, Optional
from dataclasses import dataclass


@dataclass
class ControlParameters:
    """
    控制参数数据类
    
    存储 PD 控制参数和摩擦力模型参数。
    """
    
    Kp: List[float]  # 位置增益
    Kd: List[float]  # 速度增益
    Fc: List[float]  # 库仑摩擦幅值
    k: List[float]   # tanh 斜率参数
    Fv: List[float]  # 粘性摩擦系数
    Fo: List[float]  # 常数偏置力矩
    
    def __post_init__(self):
        """验证参数"""
        n = len(self.Kp)
        if not all(len(getattr(self, attr)) == n for attr in ['Kd', 'Fc', 'k', 'Fv', 'Fo']):
            raise ValueError("所有参数向量必须具有相同的长度")
    
    @classmethod
    def from_config_loader(cls, loader, param_name: str) -> 'ControlParameters':
        """
        从配置加载器创建控制参数
        
        参数:
            loader: ConfigLoader 实例
            param_name: 参数节点名称（例如 "LeaderArmParam"）
        
        返回:
            ControlParameters 实例
        """
        return cls(
            Kp=loader.get_vector(param_name, "Kp"),
            Kd=loader.get_vector(param_name, "Kd"),
            Fc=loader.get_vector(param_name, "Fc"),
            k=loader.get_vector(param_name, "k"),
            Fv=loader.get_vector(param_name, "Fv"),
            Fo=loader.get_vector(param_name, "Fo"),
        )
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "Kp": self.Kp,
            "Kd": self.Kd,
            "Fc": self.Fc,
            "k": self.k,
            "Fv": self.Fv,
            "Fo": self.Fo,
        }


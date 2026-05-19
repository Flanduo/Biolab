"""
YAML 配置文件加载器

本模块提供了加载和解析 YAML 配置文件的功能，
用于读取控制参数（Kp、Kd、摩擦力参数等）。
"""

import os
import yaml
from typing import Dict, List, Optional
from .exceptions import ConfigurationError


class ConfigLoader:
    """
    YAML 配置文件加载器
    
    用于加载控制参数配置文件，支持读取向量和标量值。
    """
    
    def __init__(self, filepath: str):
        """
        初始化配置加载器
        
        参数:
            filepath: YAML 配置文件路径
        
        异常:
            ConfigurationError: 如果文件无法加载
        """
        if not os.path.exists(filepath):
            raise ConfigurationError(f"配置文件不存在: {filepath}")
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                self._data = yaml.safe_load(f)
            if self._data is None:
                raise ConfigurationError(f"配置文件为空: {filepath}")
        except yaml.YAMLError as e:
            raise ConfigurationError(f"无法解析 YAML 文件 {filepath}: {e}") from e
        except Exception as e:
            raise ConfigurationError(f"无法加载配置文件 {filepath}: {e}") from e
    
    def get_vector(self, node_name: str, key: str) -> List[float]:
        """
        获取向量值
        
        参数:
            node_name: 节点名称（例如 "LeaderArmParam"）
            key: 键名（例如 "Kp"）
        
        返回:
            浮点数向量
        
        异常:
            ConfigurationError: 如果节点或键不存在
        """
        if node_name not in self._data:
            raise ConfigurationError(f"节点 '{node_name}' 不存在")
        
        node = self._data[node_name]
        if key not in node:
            raise ConfigurationError(f"键 '{key}' 在节点 '{node_name}' 中不存在")
        
        value = node[key]
        if not isinstance(value, list):
            raise ConfigurationError(f"键 '{key}' 的值不是列表")
        
        try:
            return [float(v) for v in value]
        except (ValueError, TypeError) as e:
            raise ConfigurationError(f"无法将值转换为浮点数列表: {e}") from e
    
    def get_double(self, node_name: str, key: str) -> float:
        """
        获取标量值
        
        参数:
            node_name: 节点名称
            key: 键名
        
        返回:
            浮点数值
        
        异常:
            ConfigurationError: 如果节点或键不存在
        """
        if node_name not in self._data:
            raise ConfigurationError(f"节点 '{node_name}' 不存在")
        
        node = self._data[node_name]
        if key not in node:
            raise ConfigurationError(f"键 '{key}' 在节点 '{node_name}' 中不存在")
        
        value = node[key]
        try:
            return float(value)
        except (ValueError, TypeError) as e:
            raise ConfigurationError(f"无法将值转换为浮点数: {e}") from e
    
    def has(self, node_name: str, key: Optional[str] = None) -> bool:
        """
        检查节点或键是否存在
        
        参数:
            node_name: 节点名称
            key: 键名（可选）
        
        返回:
            如果存在返回 True，否则返回 False
        """
        if node_name not in self._data:
            return False
        
        if key is None:
            return True
        
        return key in self._data[node_name]
    
    def get_node(self, node_name: str) -> Dict:
        """
        获取整个节点
        
        参数:
            node_name: 节点名称
        
        返回:
            节点的字典数据
        
        异常:
            ConfigurationError: 如果节点不存在
        """
        if node_name not in self._data:
            raise ConfigurationError(f"节点 '{node_name}' 不存在")
        
        return self._data[node_name]


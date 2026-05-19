#!/usr/bin/env python3
"""
OpenArm SDK 配置加载示例

本示例演示如何从配置文件加载控制参数。
"""

import os
import sys
from pathlib import Path

# 添加 SDK 路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from openarm_sdk import ConfigLoader, ControlParameters


def main():
    """主函数"""
    
    # 配置文件路径（相对于 ros2_ws 目录）
    config_dir = Path(__file__).parent.parent.parent / "src" / "openarm_teleop" / "config"
    leader_config = config_dir / "leader.yaml"
    follower_config = config_dir / "follower.yaml"
    
    if not leader_config.exists():
        print(f"配置文件不存在: {leader_config}")
        print("请确保在 ros2_ws 目录中运行此脚本")
        return
    
    # 加载 Leader 参数
    print("=" * 50)
    print("加载 Leader 控制参数")
    print("=" * 50)
    loader = ConfigLoader(str(leader_config))
    leader_params = ControlParameters.from_config_loader(loader, "LeaderArmParam")
    
    print(f"Kp: {leader_params.Kp}")
    print(f"Kd: {leader_params.Kd}")
    print(f"Fc: {leader_params.Fc}")
    print(f"k:  {leader_params.k}")
    print(f"Fv: {leader_params.Fv}")
    print(f"Fo: {leader_params.Fo}")
    
    # 加载 Follower 参数
    print("\n" + "=" * 50)
    print("加载 Follower 控制参数")
    print("=" * 50)
    loader = ConfigLoader(str(follower_config))
    follower_params = ControlParameters.from_config_loader(loader, "FollowerArmParam")
    
    print(f"Kp: {follower_params.Kp}")
    print(f"Kd: {follower_params.Kd}")
    
    print("\n配置加载成功！")
    print("这些参数可以用于配置控制算法。")


if __name__ == "__main__":
    main()


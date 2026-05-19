#!/usr/bin/env python3
"""
OpenArm SDK 双边控制示例（框架）

本示例展示了如何实现双边控制（Leader/Follower）的框架。
完整实现需要动力学计算和状态同步。
"""

import sys
import signal
import time
from pathlib import Path

# 添加 SDK 路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from openarm_sdk import OpenArmSDK, ControlMode, ConfigLoader, ControlParameters


class BilateralControlDemo:
    """双边控制演示类（框架）"""
    
    def __init__(self, leader_can="can0", follower_can="can1"):
        """初始化双边控制"""
        # 初始化 Leader 和 Follower SDK
        self.leader_sdk = OpenArmSDK(can_interface=leader_can, enable_can_fd=True)
        self.follower_sdk = OpenArmSDK(can_interface=follower_can, enable_can_fd=True)
        
        self.running = False
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """信号处理"""
        print("\n接收到停止信号...")
        self.running = False
    
    def load_configs(self):
        """加载控制参数"""
        config_dir = Path(__file__).parent.parent.parent / "src" / "openarm_teleop" / "config"
        
        # 加载 Leader 参数
        leader_loader = ConfigLoader(str(config_dir / "leader.yaml"))
        leader_params = ControlParameters.from_config_loader(leader_loader, "LeaderArmParam")
        
        # 加载 Follower 参数
        follower_loader = ConfigLoader(str(config_dir / "follower.yaml"))
        follower_params = ControlParameters.from_config_loader(
            follower_loader, "FollowerArmParam"
        )
        
        print("配置加载成功：")
        print(f"  Leader Kp: {leader_params.Kp}")
        print(f"  Follower Kp: {follower_params.Kp}")
        
        return leader_params, follower_params
    
    def start(self):
        """启动双边控制"""
        print("=" * 60)
        print("OpenArm SDK 双边控制演示（框架）")
        print("=" * 60)
        print("注意：这是控制框架，完整实现需要动力学计算")
        print()
        
        # 初始化机械臂
        print("初始化 Leader 机械臂...")
        self.leader_sdk.init_arm_motors()
        
        print("初始化 Follower 机械臂...")
        self.follower_sdk.init_arm_motors()
        
        # 加载配置
        print("加载控制参数...")
        leader_params, follower_params = self.load_configs()
        
        print("\n双边控制框架已准备就绪")
        print("完整实现需要：")
        print("  1. 动力学计算（重力、科里奥利力）")
        print("  2. 状态同步（Leader <-> Follower）")
        print("  3. PD 控制计算")
        print("  4. 力反馈计算")
        print("\n按 Ctrl+C 退出")
        
        self.running = True
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
    
    def stop(self):
        """停止控制"""
        print("\n正在停止...")
        self.leader_sdk.close()
        self.follower_sdk.close()
        print("已停止")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="OpenArm 双边控制演示")
    parser.add_argument("--leader-can", default="can0", help="Leader CAN 接口")
    parser.add_argument("--follower-can", default="can1", help="Follower CAN 接口")
    
    args = parser.parse_args()
    
    demo = BilateralControlDemo(
        leader_can=args.leader_can,
        follower_can=args.follower_can
    )
    demo.start()


if __name__ == "__main__":
    main()


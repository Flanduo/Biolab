#!/usr/bin/env python3
"""
OpenArm SDK 重力补偿模式示例

本示例演示如何使用 SDK 进行重力补偿控制。
重力补偿模式允许用户轻松拖动机械臂，非常适合教学和调试。
"""

import sys
import os
import time
import signal
import subprocess

# 检查是否在正确的目录下运行
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CURRENT_DIR = os.getcwd()
if "openarm_sdk" in _CURRENT_DIR:
    _SDK_DIR = os.path.join(_CURRENT_DIR.split("openarm_sdk")[0], "openarm_sdk")
    if os.path.exists(_SDK_DIR):
        sys.path.insert(0, os.path.dirname(_SDK_DIR))

try:
    from openarm_sdk import OpenArmSDK, ControlMode
    from openarm_sdk.exceptions import ConnectionError
except ImportError as e:
    print("=" * 60)
    print("❌ 导入错误: 无法导入 openarm_sdk")
    print("=" * 60)
    print(f"错误信息: {e}")
    print("\n解决方案:")
    print("  1. 确保在正确目录: cd ~/ros2_ws/openarm_sdk")
    print("  2. 安装 SDK: pip3 install -e .")
    print("  3. 检查 openarm_can Python 绑定是否已构建")
    sys.exit(1)


def check_can_interface(can_interface: str) -> bool:
    """检查 CAN 接口是否存在"""
    try:
        result = subprocess.run(
            ["ip", "link", "show", can_interface],
            capture_output=True,
            text=True,
            timeout=2,
            check=False
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


class GravityCompensationDemo:
    """重力补偿演示类"""
    
    def __init__(self, can_interface="can0"):
        """初始化演示"""
        # 检查硬件
        if not check_can_interface(can_interface):
            raise ConnectionError(
                f"CAN 接口 {can_interface} 不存在。请检查硬件连接。"
            )
        
        try:
            self.sdk = OpenArmSDK(
                can_interface=can_interface,
                enable_can_fd=True,
                arm_type="v10"
            )
        except ConnectionError as e:
            raise ConnectionError(
                f"无法连接到 CAN 接口: {e}\n"
                "可能原因：接口未激活、硬件未连接或权限不足"
            ) from e
        except ImportError as e:
            raise ImportError(
                f"openarm_can Python 绑定未找到: {e}\n"
                "请先构建: cd ~/ros2_ws/src/openarm_can/python && pip3 install --no-build-isolation -e ."
            ) from e
        
        self.running = False
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """信号处理函数"""
        del signum, frame  # 未使用但必须保留签名
        print("\n接收到停止信号，正在退出...")
        self.running = False
    
    def start(self):
        """启动重力补偿模式"""
        print("初始化机械臂...")
        self.sdk.init_arm_motors()
        self.sdk.init_gripper_motor()
        
        print("启动重力补偿模式...")
        self.sdk.set_control_mode(ControlMode.GRAVITY_COMPENSATION)
        self.sdk.start_control_loop()
        
        self.running = True
        print("\n重力补偿模式已启动！")
        print("现在可以轻松拖动机械臂了。")
        print("按 Ctrl+C 停止\n")
        
        # 主循环 - 监控状态
        try:
            while self.running:
                # 可以在这里添加状态监控、日志记录等功能
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
    
    def stop(self):
        """停止重力补偿模式"""
        print("\n正在停止重力补偿模式...")
        self.sdk.stop_control_loop()
        self.sdk.close()
        print("已停止并关闭 SDK")


def main():
    """主函数"""
    can_interface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    
    print("=" * 50)
    print("OpenArm SDK 重力补偿模式演示")
    print("=" * 50)
    print(f"使用 CAN 接口: {can_interface}")
    print()
    
    # 检查硬件
    print("正在检查硬件连接...")
    if not check_can_interface(can_interface):
        print(f"❌ CAN 接口 {can_interface} 不存在")
        print("\n故障排查：")
        print(f"  1. 检查硬件连接: ip link show {can_interface}")
        print("  2. 检查驱动是否加载")
        return
    print("✓ CAN 接口存在")
    print("\n注意：确保机械臂已经完成底层初始化")
    print("（运行 init_motors.sh 脚本完成初始化）\n")
    
    try:
        demo = GravityCompensationDemo(can_interface=can_interface)
        demo.start()
    except (ConnectionError, ImportError) as e:
        print(f"❌ 初始化失败: {e}")
        return


if __name__ == "__main__":
    main()


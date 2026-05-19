#!/usr/bin/env python3
"""
OpenArm SDK 的高级控制示例

本示例演示 SDK 的高级应用场景，面向上层开发人员：
- 重力补偿模式
- 状态监控
- 任务级别的操作

注意：底层初始化（使能电机、设置零位等）应该通过 init_motors.sh 脚本完成，
SDK 专注于提供高级控制功能。
"""

import sys
import os
import time
import subprocess

# 确保从源码目录直接运行 examples 时，也能导入 openarm_sdk 包
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)  # .../openarm_sdk
_PKG_INIT = os.path.join(_REPO_ROOT, "openarm_sdk", "__init__.py")
if os.path.exists(_PKG_INIT):
    # 需要加入 SDK 工程根目录（包含 openarm_sdk/ 子目录）
    if _REPO_ROOT not in sys.path:
        sys.path.insert(0, _REPO_ROOT)

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


def check_hardware(can_interface: str) -> tuple[bool, str]:
    """检查硬件连接状态"""
    # 检查 CAN 接口是否存在
    if not check_can_interface(can_interface):
        return False, f"CAN 接口 {can_interface} 不存在。请检查硬件连接。"
    
    # 检查是否可以打开接口（需要实际尝试连接）
    return True, ""


def main():
    """演示高级控制的主函数"""
    
    # 解析命令行参数
    can_interface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    
    print("=" * 60)
    print("OpenArm SDK 高级控制示例")
    print("=" * 60)
    print(f"使用 CAN 接口: {can_interface}\n")
    
    # 检查硬件连接
    print("正在检查硬件连接...")
    hardware_ok, error_msg = check_hardware(can_interface)
    if not hardware_ok:
        print(f"❌ 硬件检查失败: {error_msg}")
        print("\n故障排查：")
        print(f"  1. 检查 CAN 接口是否存在: ip link show {can_interface}")
        print("  2. 检查硬件连接和驱动")
        print("  3. 如果接口存在但处于 DOWN 状态，运行: sudo ip link set can0 up")
        return
    
    print("✓ CAN 接口存在")
    print(f"注意: 确保底层已初始化（运行 init_motors.sh）")
    print()
    
    # 初始化 SDK（假设底层已经初始化完成）
    print("正在初始化 OpenArm SDK...")
    try:
        sdk = OpenArmSDK(
            can_interface=can_interface,
            enable_can_fd=True,
            arm_type="v10"
        )
        print("✓ SDK 初始化成功")
    except ConnectionError as e:
        print(f"❌ CAN 连接失败: {e}")
        print("\n可能的原因：")
        print("  1. CAN 接口未激活（运行: sudo ip link set can0 up）")
        print("  2. 硬件未连接或驱动问题")
        print("  3. 权限不足（尝试: sudo usermod -a -G dialout $USER，然后重新登录）")
        return
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("\n解决方案：")
        print("  1. 构建 openarm_can Python 绑定:")
        print("     cd ~/ros2_ws/src/openarm_can/python")
        print("     pip3 install --no-build-isolation -e .")
        return
    except Exception as e:
        print(f"❌ SDK 初始化失败: {e}")
        print("\n故障排查：")
        print("  1. 检查 openarm_can Python 绑定是否已构建")
        print("  2. 检查 CAN 接口状态: ip link show can0")
        print("  3. 检查权限（可能需要 sudo 或加入 dialout 组）")
        return
    
    try:
        # 初始化机械臂电机（使用默认配置）
        print("正在初始化机械臂电机...")
        sdk.init_arm_motors()
        
        # 初始化夹爪电机
        print("正在初始化夹爪电机...")
        sdk.init_gripper_motor()
        
        # 示例 1: 启动重力补偿模式
        print("\n=== 示例 1: 重力补偿模式 ===")
        print("启动重力补偿模式，机械臂可以轻松拖动...")
        sdk.set_control_mode(ControlMode.GRAVITY_COMPENSATION)
        sdk.start_control_loop()
        
        # 运行一段时间
        print("重力补偿模式运行中，可以手动拖动机械臂...")
        print("按 Ctrl+C 停止")
        try:
            time.sleep(10)  # 运行 10 秒
        except KeyboardInterrupt:
            print("\n用户中断")
        
        sdk.stop_control_loop()
        print("重力补偿模式已停止")
        
        # 示例 2: 获取机械臂状态
        print("\n=== 示例 2: 状态监控 ===")
        arm = sdk.get_arm()
        motors = arm.get_motors()
        
        print(f"电机数量: {len(motors)}")
        for i, motor in enumerate(motors):
            print(f"  电机 {i+1}: 位置={motor.get_position():.3f} rad, "
                  f"速度={motor.get_velocity():.3f} rad/s")
        
        # 示例 3: 夹爪操作
        print("\n=== 示例 3: 夹爪控制 ===")
        gripper = sdk.get_gripper()
        print("张开夹爪...")
        gripper.open(kp=50.0, kd=1.0)
        time.sleep(2)
        
        print("闭合夹爪...")
        gripper.close(kp=50.0, kd=1.0)
        time.sleep(2)
        
        print("\n所有示例完成！")
        print("SDK 提供了高级控制接口，便于快速开发应用。")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        sdk.close()
        print("\nSDK 已关闭。")


if __name__ == "__main__":
    main()


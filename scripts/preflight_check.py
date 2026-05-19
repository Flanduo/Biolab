#!/usr/bin/env python3
"""
Biolab 全流程启动检查脚本
逐项检查硬件子系统是否就绪，确认后可启动全流程

用法:
  python3 scripts/preflight_check.py              # 默认只检查灵巧手
  python3 scripts/preflight_check.py --all         # 检查全部
  python3 scripts/preflight_check.py --hands       # 只检查灵巧手
  python3 scripts/preflight_check.py --camera      # 只检查相机
"""

import sys
import os
import subprocess
import argparse

# LinkerHand SDK 路径
LINKERHAND_SDK = "/home/elwg/Biolab/linkerhand-sdk"
LINKERHAND_SDK_INNER = f"{LINKERHAND_SDK}/LinkerHand"


# ─── CAN 接口 ───

def _can_is_up(iface):
    """检查单个 CAN 接口是否 UP"""
    result = subprocess.run(["ip", "link", "show", iface], capture_output=True, text=True)
    return "UP" in result.stdout and result.returncode == 0


def setup_can_interfaces(interfaces, bitrate=1000000):
    """检查并自动拉起 CAN 接口（需要 sudo 密码）"""
    need_setup = [iface for iface in interfaces if not _can_is_up(iface)]
    if not need_setup:
        for iface in interfaces:
            print(f"  ✅ {iface}: UP")
        return True

    for iface in need_setup:
        print(f"  {iface}: DOWN, 正在拉起 (需要 sudo 密码)...")
        # 先 down 再 up，避免 busy 错误
        subprocess.run(["sudo", "ip", "link", "set", iface, "down"],
                        stdin=sys.stdin, capture_output=True)
        r = subprocess.run(
            ["sudo", "ip", "link", "set", iface, "up", "type", "can", "bitrate", str(bitrate)],
            stdin=sys.stdin,  # 继承终端，让 sudo 能读密码
        )
        if r.returncode == 0 and _can_is_up(iface):
            print(f"  ✅ {iface}: UP (已拉起)")
        else:
            print(f"  ❌ {iface}: 拉起失败")
            return False

    return True


# ─── LinkerHand SDK 环境准备 ───

def _setup_linkerhand_env():
    """配置 SDK 导入环境，返回原始 CWD"""
    original_cwd = os.getcwd()
    for p in [LINKERHAND_SDK_INNER, LINKERHAND_SDK]:
        if p in sys.path:
            sys.path.remove(p)
        sys.path.insert(0, p)
    # 移除可能冲突的 utils 模块缓存
    for key in list(sys.modules.keys()):
        if key == 'utils' or key.startswith('utils.'):
            del sys.modules[key]
    os.chdir(LINKERHAND_SDK_INNER)
    return original_cwd


def _restore_env(original_cwd):
    os.chdir(original_cwd)


# ─── 灵巧手检查 ───

def check_single_hand(hand_type, can_interface):
    """检查单个灵巧手，返回 (ok, info_dict)"""
    from LinkerHand.linker_hand_api import LinkerHandApi

    side = "左" if hand_type == "left" else "右"
    info = {}

    try:
        hand = LinkerHandApi(
            hand_type=hand_type,
            hand_joint="O6",
            can=can_interface
        )

        # SDK 构造函数已自动读取版本和序列号
        info["serial"] = getattr(hand, "serial_number", "未知")
        info["version"] = hand.get_embedded_version()

        state = hand.get_state()
        info["state"] = state
        # O6 有 6 个关节，全 254 = 完全张开
        if state and len(state) == 6:
            all_open = all(v >= 250 for v in state)
            all_closed = all(v <= 5 for v in state)
            if all_open:
                info["state_desc"] = "张开"
            elif all_closed:
                info["state_desc"] = "握拳"
            else:
                info["state_desc"] = "半握"
        else:
            info["state_desc"] = "未知"

        try:
            info["temperature"] = hand.get_temperature()
        except Exception:
            info["temperature"] = None

        try:
            info["fault"] = hand.get_fault()
        except Exception:
            info["fault"] = None

        hand.close_can()
        return True, info

    except SystemExit:
        return False, {"error": "CAN 接口未打开或设备无响应"}
    except Exception as e:
        return False, {"error": str(e)}


def check_hands():
    """检查灵巧手子系统"""
    print("=" * 50)
    print("  灵巧手检查")
    print("=" * 50)

    # Step 1: CAN 接口
    print("\n[1/2] CAN 接口 (can3=左手, can2=右手)")
    can_ok = setup_can_interfaces(["can2", "can3"])
    if not can_ok:
        return False

    # Step 2: 连接灵巧手
    print("\n[2/2] 连接灵巧手")
    original_cwd = _setup_linkerhand_env()

    hands = [
        ("left", "can3", "左手"),
        ("right", "can2", "右手"),
    ]

    results = {}
    for hand_type, can_iface, label in hands:
        print(f"\n  ── {label} O6 ({can_iface}) ──")
        ok, info = check_single_hand(hand_type, can_iface)
        results[label] = ok

        if ok:
            print(f"    序列号:   {info.get('serial', '?')}")
            print(f"    固件版本: {info.get('version', '?')}")
            print(f"    当前状态: {info.get('state', '?')} ({info.get('state_desc', '?')})")
            if info.get("temperature") is not None:
                print(f"    温度:     {info['temperature']}")
            if info.get("fault") is not None:
                fault = info["fault"]
                fault_ok = all(f == 0 for f in fault) if fault else True
                print(f"    故障码:   {fault} {'✅ 无故障' if fault_ok else '⚠️  存在故障'}")
            print(f"    ✅ {label}就绪")
        else:
            err = info.get("error", "未知错误")
            print(f"    ❌ {label}检查失败: {err}")

    _restore_env(original_cwd)

    all_ok = all(results.values())
    print(f"\n  灵巧手总览: {'✅ 全部就绪' if all_ok else '❌ 存在问题'}")
    return all_ok


# ─── 相机检查 ───

def check_camera():
    """检查 ZED 相机服务"""
    print("=" * 50)
    print("  相机检查")
    print("=" * 50)

    try:
        import requests
    except ImportError:
        print("  ❌ 缺少 requests 库")
        return False

    try:
        r = requests.get("http://localhost:5050/status", timeout=3)
        if r.status_code == 200:
            status = r.json()
            print(f"  状态: 在线")
            print(f"  分辨率: {status.get('resolution', '?')}")
            print(f"  帧率: {status.get('fps', '?')} FPS")
            print(f"  深度模式: {status.get('depth_mode', '?')}")

            # 读内参
            try:
                ri = requests.get("http://localhost:5050/intrinsics", timeout=3)
                if ri.status_code == 200:
                    intr = ri.json()
                    print(f"  内参: fx={intr.get('fx', '?')}, fy={intr.get('fy', '?')}")
            except Exception:
                pass

            print(f"\n  ✅ 相机服务就绪")
            return True
        else:
            print(f"  ❌ 相机服务响应异常: HTTP {r.status_code}")
            return False
    except requests.ConnectionError:
        print("  ❌ 相机服务未启动")
        print("    启动命令: ~/Biolab/ZEDProject/start_capture.sh")
        return False
    except Exception as e:
        print(f"  ❌ 检查失败: {e}")
        return False


# ─── 主入口 ───

def main():
    parser = argparse.ArgumentParser(
        description="Biolab 全流程启动检查",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python3 scripts/preflight_check.py              # 检查灵巧手\n"
               "  python3 scripts/preflight_check.py --all         # 检查全部\n"
               "  python3 scripts/preflight_check.py --camera      # 只检查相机\n"
    )
    parser.add_argument("--hands", action="store_true", help="检查灵巧手")
    parser.add_argument("--camera", action="store_true", help="检查 ZED 相机服务")
    parser.add_argument("--all", action="store_true", help="检查全部子系统")
    args = parser.parse_args()

    # 默认只检查灵巧手
    if not any([args.hands, args.camera, args.all]):
        args.hands = True

    results = {}

    if args.hands or args.all:
        results["灵巧手"] = check_hands()
        print()

    if args.camera or args.all:
        results["相机"] = check_camera()
        print()

    # 汇总
    print("=" * 50)
    print("  检查结果汇总")
    print("=" * 50)
    for name, ok in results.items():
        print(f"  {name}: {'✅ 就绪' if ok else '❌ 未就绪'}")

    all_ok = all(results.values())
    print(f"\n  {'✅ 全部就绪，可以启动全流程' if all_ok else '❌ 存在问题，请先修复'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

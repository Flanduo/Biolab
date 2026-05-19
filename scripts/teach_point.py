#!/usr/bin/env python3
"""
打点控制 - 录制 & 复现机械臂 + 灵巧手关节位置

用法:
  source /opt/ros/humble/setup.bash
  source ~/Biolab/ros2_ws/install/setup.bash

  # 录制（交互式，按 Enter 记录点）
  python3 teach_point.py record [-o output.json] [--no-hand]

  # 复现（按顺序移动到每个记录点）
  python3 teach_point.py replay -i input.json [--speed 0.5] [--no-hand]

  # 列出已保存的录制文件
  python3 teach_point.py list
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import json
import time
import os
import sys
import argparse
import threading
from datetime import datetime

# ============== 配置 ==============
OUTPUT_DIR = "/home/elwg/Biolab/configs/teach_points"
SDK_PATH = "/home/elwg/Biolab/linkerhand-sdk"
SDK_LINKERHAND = f"{SDK_PATH}/LinkerHand"

HAND_CONFIG = {
    "left":  {"can": "can3", "joint": "O6"},
    "right": {"can": "can2", "joint": "O6"},
}


# ============== LinkerHand 初始化 ==============
def init_hands():
    """初始化灵巧手，返回 {side: api}"""
    hands = {}

    for p in [SDK_LINKERHAND, SDK_PATH]:
        if p not in sys.path:
            sys.path.insert(0, p)

    # 清除 utils 模块冲突
    for key in list(sys.modules.keys()):
        if key == 'utils' or key.startswith('utils.'):
            del sys.modules[key]

    original_cwd = os.getcwd()
    os.chdir(SDK_LINKERHAND)

    try:
        from LinkerHand.linker_hand_api import LinkerHandApi

        for side, cfg in HAND_CONFIG.items():
            try:
                print(f"[手部] 连接 {side} 手: {cfg['joint']} CAN:{cfg['can']}...")
                api = LinkerHandApi(
                    hand_type=side, hand_joint=cfg["joint"], can=cfg["can"])
                hands[side] = api
                print(f"[手部] {side} 手连接成功")
            except SystemExit:
                print(f"[手部] {side} 手连接失败: CAN 接口不可用")
            except Exception as e:
                print(f"[手部] {side} 手连接失败: {e}")
    except ImportError as e:
        print(f"[手部] SDK 导入失败: {e}")
    finally:
        os.chdir(original_cwd)

    return hands


def read_hand_states(hands):
    """读取灵巧手当前状态"""
    result = {}
    for side in ["left", "right"]:
        api = hands.get(side)
        if api is None:
            result[side] = None
            continue
        try:
            state = api.get_state()
            result[side] = list(state[:6]) if state and len(state) >= 6 else None
        except Exception:
            result[side] = None
    return result


def move_hand(hands, side, pose):
    """控制灵巧手"""
    api = hands.get(side)
    if api is None or pose is None:
        return
    try:
        api.finger_move(pose)
    except Exception as e:
        print(f"[手部] {side} 手移动失败: {e}")


# ============== ROS2 Node ==============
class TeachPointNode(Node):
    def __init__(self):
        super().__init__('teach_point')

        self._arm_state = {}
        self._lock = threading.Lock()

        self.sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        self.pub_left = self.create_publisher(
            Float64MultiArray, '/left_forward_position_controller/commands', 10)
        self.pub_right = self.create_publisher(
            Float64MultiArray, '/right_forward_position_controller/commands', 10)

    def _joint_cb(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._arm_state[name] = pos

    def get_arm_positions(self):
        """获取当前臂关节位置（7+7）"""
        with self._lock:
            left = [self._arm_state.get(f"openarm_left_joint{i}", 0.0) for i in range(1, 8)]
            right = [self._arm_state.get(f"openarm_right_joint{i}", 0.0) for i in range(1, 8)]
            return left, right

    def publish_arm(self, left, right):
        """发布臂关节命令"""
        msg_l = Float64MultiArray()
        msg_l.data = [float(v) for v in left]
        self.pub_left.publish(msg_l)

        msg_r = Float64MultiArray()
        msg_r.data = [float(v) for v in right]
        self.pub_right.publish(msg_r)

    def wait_for_state(self, timeout=5.0):
        """等待关节状态到达"""
        print("等待关节状态...")
        start = time.time()
        while time.time() - start < timeout:
            left, right = self.get_arm_positions()
            if any(v != 0.0 for v in left) or any(v != 0.0 for v in right):
                print("关节状态已就绪")
                return True
            time.sleep(0.1)
        print("警告: 未检测到关节状态，可能 ROS2 驱动未启动")
        return False


# ============== 录制 ==============
def do_record(args, hands, node):
    points = []

    print("\n========== 录制模式 ==========")
    print("  Enter   记录当前点")
    print("  n 名字  记录并命名")
    print("  d       删除上一个点")
    print("  p       打印已记录点")
    print("  q       保存并退出")
    print("=" * 32)

    while True:
        try:
            cmd = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd in ('q', 'quit', 'exit'):
            break

        elif cmd == '' or cmd == 'r':
            _record_point(points, hands, node, None)

        elif cmd.startswith('n ') or cmd.startswith('name '):
            name = cmd.split(None, 1)[1].strip() if len(cmd.split()) > 1 else None
            _record_point(points, hands, node, name)

        elif cmd == 'd':
            if points:
                removed = points.pop()
                print(f"  删除点: {removed['name']}")
            else:
                print("  没有点可删除")

        elif cmd == 'p':
            _print_points(points)
        else:
            print("  未知命令")

    if not points:
        print("没有记录任何点")
        return

    # 保存
    data = {
        "metadata": {
            "created": datetime.now().isoformat(),
            "num_points": len(points),
            "arm_joints": 7,
            "hand_type": "O6",
            "hand_joints": 6,
        },
        "points": points,
    }

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = args.output or os.path.join(OUTPUT_DIR, f"teach_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"\n已保存 {len(points)} 个点到 {path}")


def _record_point(points, hands, node, name):
    """记录一个点"""
    left_arm, right_arm = node.get_arm_positions()
    hand_states = read_hand_states(hands)

    point = {
        "name": name or f"point_{len(points) + 1}",
        "left_arm": [round(v, 4) for v in left_arm],
        "right_arm": [round(v, 4) for v in right_arm],
    }
    for side in ["left", "right"]:
        h = hand_states.get(side)
        point[f"{side}_hand"] = [int(v) for v in h] if h else None

    points.append(point)
    idx = len(points)
    print(f"  #{idx} {point['name']}")
    print(f"    左臂: {[round(v, 2) for v in left_arm]}")
    print(f"    右臂: {[round(v, 2) for v in right_arm]}")
    for side in ["left", "right"]:
        h = point[f"{side}_hand"]
        if h:
            print(f"    {side}手: {h}")


def _print_points(points):
    if not points:
        print("  没有已记录的点")
        return
    for i, pt in enumerate(points):
        print(f"  #{i + 1} {pt['name']}")
        print(f"    左臂: {[round(v, 2) for v in pt['left_arm']]}")
        print(f"    右臂: {[round(v, 2) for v in pt['right_arm']]}")


# ============== 复现 ==============
def do_replay(args, hands, node):
    path = args.input
    if not os.path.exists(path):
        print(f"文件不存在: {path}")
        return

    with open(path) as f:
        data = json.load(f)

    points = data.get("points", [])
    if not points:
        print("没有可复现的点")
        return

    speed = args.speed

    print(f"\n========== 复现模式 ==========")
    print(f"  文件: {path}")
    print(f"  点数: {len(points)}")
    print(f"  速度: {speed} rad/s")
    _print_points(points)
    print("=" * 32)

    try:
        input("按 Enter 开始复现 (Ctrl+C 取消)...")
    except KeyboardInterrupt:
        print("\n已取消")
        return

    for i, point in enumerate(points):
        name = point.get("name", f"point_{i + 1}")
        print(f"\n--- [{i + 1}/{len(points)}] {name} ---")

        target_left = point["left_arm"]
        target_right = point["right_arm"]

        # 手：同时发送命令
        for side in ["left", "right"]:
            hand_pose = point.get(f"{side}_hand")
            if hand_pose and hands.get(side):
                move_hand(hands, side, hand_pose)
                print(f"  {side}手 -> {hand_pose}")

        # 臂：线性插值
        current_left, current_right = node.get_arm_positions()
        _interpolate_move(node, current_left, current_right,
                          target_left, target_right, speed)

        print(f"  到达 {name}")
        time.sleep(0.3)

    print("\n========== 复现完成 ==========")


def _interpolate_move(node, cur_left, cur_right, tgt_left, tgt_right, speed, hz=50):
    """线性插值移动臂"""
    dt = 1.0 / hz

    # 最大关节角位移决定运动时长
    max_delta = 0.0
    for c, t in zip(cur_left + cur_right, tgt_left + tgt_right):
        max_delta = max(max_delta, abs(t - c))

    if max_delta < 0.005:
        node.publish_arm(tgt_left, tgt_right)
        return

    duration = max_delta / speed
    steps = max(1, int(duration / dt))

    for step in range(1, steps + 1):
        ratio = step / steps
        left = [c + (t - c) * ratio for c, t in zip(cur_left, tgt_left)]
        right = [c + (t - c) * ratio for c, t in zip(cur_right, tgt_right)]
        node.publish_arm(left, right)
        time.sleep(dt)

    node.publish_arm(tgt_left, tgt_right)


# ============== 列表 ==============
def do_list(_args):
    if not os.path.exists(OUTPUT_DIR):
        print(f"目录不存在: {OUTPUT_DIR}")
        return

    files = sorted(f for f in os.listdir(OUTPUT_DIR) if f.endswith(".json"))
    if not files:
        print("没有已保存的录制文件")
        return

    print(f"\n已保存的录制 ({OUTPUT_DIR}):\n")
    for fname in files:
        path = os.path.join(OUTPUT_DIR, fname)
        with open(path) as f:
            data = json.load(f)
        meta = data.get("metadata", {})
        pts = data.get("points", [])
        print(f"  {fname}")
        print(f"    点数: {len(pts)}  创建: {meta.get('created', '?')}")
        for j, pt in enumerate(pts[:3]):
            print(f"    #{j + 1} {pt.get('name', '?')}")
        if len(pts) > 3:
            print(f"    ... 还有 {len(pts) - 3} 个点")
        print()


# ============== Main ==============
def main():
    parser = argparse.ArgumentParser(description="打点控制 - 录制 & 复现")
    sub = parser.add_subparsers(dest="command")

    rec = sub.add_parser("record", help="录制模式")
    rec.add_argument("-o", "--output", type=str, default=None, help="输出文件路径")
    rec.add_argument("--no-hand", action="store_true", help="不连接灵巧手")

    rep = sub.add_parser("replay", help="复现模式")
    rep.add_argument("-i", "--input", type=str, required=True, help="输入 JSON 文件")
    rep.add_argument("--speed", type=float, default=0.5, help="臂运动速度 rad/s (默认 0.5)")
    rep.add_argument("--no-hand", action="store_true", help="不控制灵巧手")

    sub.add_parser("list", help="列出已保存的录制文件")

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        return

    if args.command == "list":
        do_list(args)
        return

    # record / replay 都需要 ROS2
    rclpy.init()
    node = TeachPointNode()

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    node.wait_for_state(timeout=5.0)

    # 初始化灵巧手
    hands = {}
    if not args.no_hand:
        hands = init_hands()

    try:
        if args.command == "record":
            do_record(args, hands, node)
        elif args.command == "replay":
            do_replay(args, hands, node)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for side, api in hands.items():
            try:
                api.close_can()
            except Exception:
                pass


if __name__ == "__main__":
    main()

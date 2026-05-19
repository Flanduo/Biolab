#!/usr/bin/env python3
"""
保存/读取 OpenArm 机械臂关节位置

用法:
  python3 scripts/save_position.py save [名称]      # 保存当前位置
  python3 scripts/save_position.py list             # 列出所有已保存位置
  python3 scripts/save_position.py restore [名称]   # 恢复到已保存位置
"""

import sys
import os
import json
import time
import subprocess

SAVE_DIR = os.path.expanduser("~/Biolab/configs/saved_positions")


def get_joint_states():
    """读取当前关节状态"""
    result = subprocess.run(
        ["ros2", "topic", "echo", "/joint_states", "--once"],
        capture_output=True, text=True, timeout=10
    )

    names = []
    positions = []
    in_names = False
    in_positions = False

    for line in result.stdout.strip().split("\n"):
        line = line.strip()
        if line.startswith("name:"):
            in_names = True
            in_positions = False
            continue
        if line.startswith("position:"):
            in_positions = True
            in_names = False
            continue
        if line.startswith("velocity:") or line.startswith("effort:"):
            in_names = False
            in_positions = False
            continue
        if in_names and line.startswith("- "):
            names.append(line[2:])
        if in_positions and line.startswith("- "):
            positions.append(float(line[2:]))

    return dict(zip(names, positions))


def save_position(name="default"):
    """保存当前位置"""
    data = get_joint_states()

    left_arm = {k: v for k, v in data.items() if "left" in k and "finger" not in k}
    right_arm = {k: v for k, v in data.items() if "right" in k and "finger" not in k}
    left_finger = {k: v for k, v in data.items() if "left" in k and "finger" in k}
    right_finger = {k: v for k, v in data.items() if "right" in k and "finger" in k}

    save = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "left_arm": left_arm,
        "right_arm": right_arm,
        "left_finger": left_finger,
        "right_finger": right_finger,
        "all_joints": data,
    }

    os.makedirs(SAVE_DIR, exist_ok=True)
    path = os.path.join(SAVE_DIR, f"{name}.json")
    with open(path, "w") as f:
        json.dump(save, f, indent=2, ensure_ascii=False)

    print(f"已保存位置 '{name}' -> {path}")
    print(f"  左臂: {list(left_arm.values())}")
    print(f"  右臂: {list(right_arm.values())}")
    if left_finger:
        print(f"  左夹爪: {list(left_finger.values())}")
    if right_finger:
        print(f"  右夹爪: {list(right_finger.values())}")


def list_positions():
    """列出所有已保存位置"""
    if not os.path.exists(SAVE_DIR):
        print("没有已保存的位置")
        return

    files = sorted(f for f in os.listdir(SAVE_DIR) if f.endswith(".json"))
    if not files:
        print("没有已保存的位置")
        return

    print(f"已保存的位置 ({SAVE_DIR}):\n")
    for fname in files:
        name = fname[:-5]
        with open(os.path.join(SAVE_DIR, fname)) as f:
            data = json.load(f)
        ts = data.get("timestamp", "?")
        left = list(data.get("left_arm", {}).values())
        right = list(data.get("right_arm", {}).values())
        print(f"  {name:20s}  ({ts})")
        print(f"    左臂: {[round(v, 3) for v in left]}")
        print(f"    右臂: {[round(v, 3) for v in right]}")
        print()


def restore_position(name="default"):
    """恢复到已保存位置"""
    path = os.path.join(SAVE_DIR, f"{name}.json")
    if not os.path.exists(path):
        print(f"位置 '{name}' 不存在")
        list_positions()
        return

    with open(path) as f:
        data = json.load(f)

    left_arm = data.get("left_arm", {})
    right_arm = data.get("right_arm", {})

    left_positions = list(left_arm.values())
    right_positions = list(right_arm.values())

    print(f"恢复位置 '{name}' ({data.get('timestamp', '?')})")
    print(f"  左臂: {left_positions}")
    print(f"  右臂: {right_positions}")

    # 发送到 forward_position_controller
    try:
        import requests
        msg = {
            "left": left_positions,
            "right": right_positions,
        }
        print("\n发送命令...")

        # 使用 ros2 topic pub
        left_cmd = json.dumps(left_positions)
        right_cmd = json.dumps(right_positions)

        subprocess.run(
            ["ros2", "topic", "pub", "--once",
             "/left_forward_position_controller/commands",
             "std_msgs/msg/Float64MultiArray",
             f"{{data: {left_cmd}}}"],
            timeout=5
        )
        subprocess.run(
            ["ros2", "topic", "pub", "--once",
             "/right_forward_position_controller/commands",
             "std_msgs/msg/Float64MultiArray",
             f"{{data: {right_cmd}}}"],
            timeout=5
        )
        print("命令已发送")

    except Exception as e:
        print(f"发送失败: {e}")
        print("请确保 ROS2 驱动已启动且 forward_position_controller 已激活")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    cmd = sys.argv[1]

    if cmd == "save":
        name = sys.argv[2] if len(sys.argv) > 2 else "default"
        save_position(name)
    elif cmd == "list":
        list_positions()
    elif cmd == "restore":
        name = sys.argv[2] if len(sys.argv) > 2 else "default"
        restore_position(name)
    else:
        print(f"未知命令: {cmd}")
        print(__doc__)


if __name__ == "__main__":
    main()

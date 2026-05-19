#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP 手套数据接收 + LinkerHand 控制脚本
- 端口 5555 接收 UDE 手套角度数据
- 将角度(弧度)转换为 LinkerHand 关节位置(0-255)
- 通过 LinkerHand SDK 发送控制指令
"""

import os
os.environ["PYTHONUNBUFFERED"] = "1"

import sys
import socket
import json
import time
import csv
import math
import signal
import threading

# 添加 LinkerHand SDK 路径
SDK_PATH = "/home/elwg/Biolab/linkerhand-sdk"
sys.path.append(SDK_PATH)

from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.mapping import (
    arc_to_range_left as _arc_to_range_left,
    arc_to_range_right as _arc_to_range_right,
    scale_value, is_within_range,
    l6_l_min, l6_l_max, l6_l_derict,
    l6_r_min, l6_r_max, l6_r_derict,
)



# === 拇指侧摆(索引1)输出重映射 ===
# SDK derict=-1: 弧度越大→输出越小 (0°→255, 80°→0)
# 手套实际活动范围: l3/r3 约 5°~50° → SDK输出约 158~238
# 重映射: 将窄区间拉伸到 0-255
THUMB_YAW_REMAP = {
    "left":  {"in_min": 0, "in_max": 255},
    "right": {"in_min": 0, "in_max": 255},
}

def remap_thumb_yaw(positions, side):
    """对拇指侧摆(索引1)做输出重映射"""
    cfg = THUMB_YAW_REMAP.get(side)
    if cfg and len(positions) > 1:
        raw = positions[1]
        in_min, in_max = cfg["in_min"], cfg["in_max"]
        if in_max != in_min:
            remapped = (raw - in_min) / (in_max - in_min) * 255.0
            positions[1] = max(0, min(255, int(remapped)))
    return positions

# === 拇指弯曲/侧摆限幅 ===
# L6/O6 索引0=拇指弯曲(max 178), 索引1=拇指侧摆(max 122)
THUMB_CLAMP = {
    0: 255,   # 拇指弯曲最大值
    1: 255,   # 拇指侧摆最大值
}
THUMB_LIMITED_JOINTS = {"L6", "O6"}

def clamp_thumb(positions, hand_joint):
    """仅对 L6/O6 的拇指弯曲(索引0)和拇指侧摆(索引1)做限幅"""
    if hand_joint.upper() not in THUMB_LIMITED_JOINTS:
        return positions
    for idx, max_val in THUMB_CLAMP.items():
        if idx < len(positions):
            positions[idx] = min(positions[idx], max_val)
    return positions


def _arc_to_range_l6(arc_values, l_min, l_max, l_derict):
    """SDK 的 arc_to_range 函数遗漏了 L6，这里补充实现"""
    hand_range = [0] * 6
    for i in range(6):
        val = is_within_range(arc_values[i], l_min[i], l_max[i])
        if l_derict[i] == -1:
            hand_range[i] = scale_value(val, l_min[i], l_max[i], 255, 0)
        else:
            hand_range[i] = scale_value(val, l_min[i], l_max[i], 0, 255)
    return hand_range


def arc_to_range_left(arc_values, hand_joint):
    """封装左手弧度转换，补充 L6 支持"""
    if hand_joint in ["L6", "O6"]:
        return _arc_to_range_l6(arc_values, l6_l_min, l6_l_max, l6_l_derict)
    return _arc_to_range_left(arc_values, hand_joint)


def arc_to_range_right(arc_values, hand_joint):
    """封装右手弧度转换，补充 L6 支持"""
    if hand_joint in ["L6", "O6"]:
        return _arc_to_range_l6(arc_values, l6_r_min, l6_r_max, l6_r_derict)
    return _arc_to_range_right(arc_values, hand_joint)

# ============================================================
# 手套参数 -> LinkerHand 关节映射表
#
# 手套参数说明（以左手 l 前缀为例，右手同理用 r 前缀）:
#   l0:  拇指第三关节俯仰角    l1:  拇指第二关节俯仰角
#   l2:  拇指第一关节俯仰角    l3:  拇指第一关节偏航角
#   l4:  食指第三关节俯仰角    l5:  食指第二关节俯仰角
#   l6:  食指第一关节俯仰角    l7:  食指第一关节偏航角
#   l8:  中指第三关节俯仰角    l9:  中指第二关节俯仰角
#   l10: 中指第一关节俯仰角    l11: 中指第一关节偏航角
#   l12: 无名指第三关节俯仰角  l13: 无名指第二关节俯仰角
#   l14: 无名指第一关节俯仰角  l15: 无名指第一关节偏航角
#   l16: 小指第三关节俯仰角    l17: 小指第二关节俯仰角
#   l18: 小指第一关节俯仰角    l19: 小指第一关节偏航角
#   l20: 拇指第一关节旋转角    l21: 食指第一关节旋转角
#   l22: 小指第一关节旋转角
# ============================================================

# --- 左手映射 ---

# L6/O6: [大拇指弯曲, 大拇指横摆, 食指弯曲, 中指弯曲, 无名指弯曲, 小拇指弯曲]
GLOVE_TO_L6 = [
    "l2",   # → L6[0] 大拇指弯曲   ← 拇指第一关节俯仰角
    "l20",  # → L6[1] 拇指旋转角   ← 拇指第一关节偏航角
    "l6",   # → L6[2] 食指弯曲     ← 食指第一关节俯仰角
    "l10",  # → L6[3] 中指弯曲     ← 中指第一关节俯仰角
    "l14",  # → L6[4] 无名指弯曲   ← 无名指第一关节俯仰角
    "l18",  # → L6[5] 小拇指弯曲   ← 小指第一关节俯仰角
]

# L7: [大拇指弯曲, 大拇指横摆, 食指弯曲, 中指弯曲, 无名指弯曲, 小拇指弯曲, 拇指旋转]
GLOVE_TO_L7 = [
    "l2",   # → L7[0] 大拇指弯曲   ← 拇指第一关节俯仰角
    "l20",  # → L6/7[1] 大拇指横摆   ← 拇指第一关节偏航角
    "l6",   # → L7[2] 食指弯曲     ← 食指第一关节俯仰角
    "l10",  # → L7[3] 中指弯曲     ← 中指第一关节俯仰角
    "l14",  # → L7[4] 无名指弯曲   ← 无名指第一关节俯仰角
    "l18",  # → L7[5] 小拇指弯曲   ← 小指第一关节俯仰角
    "l20",  # → L7[6] 拇指旋转     ← 拇指第一关节旋转角
]

# L10: [拇指根部, 拇指侧摆, 食指根部, 中指根部, 无名指根部, 小指根部, 食指侧摆, 无名指侧摆, 小指侧摆, 拇指旋转]
GLOVE_TO_L10 = [
    "l2",   # → L10[0] 拇指根部     ← 拇指第一关节俯仰角
    "l3",   # → L10[1] 拇指侧摆     ← 拇指第一关节偏航角
    "l6",   # → L10[2] 食指根部     ← 食指第一关节俯仰角
    "l10",  # → L10[3] 中指根部     ← 中指第一关节俯仰角
    "l14",  # → L10[4] 无名指根部   ← 无名指第一关节俯仰角
    "l18",  # → L10[5] 小指根部     ← 小指第一关节俯仰角
    "l7",   # → L10[6] 食指侧摆     ← 食指第一关节偏航角
    "l15",  # → L10[7] 无名指侧摆   ← 无名指第一关节偏航角
    "l19",  # → L10[8] 小指侧摆     ← 小指第一关节偏航角
    "l20",  # → L10[9] 拇指旋转     ← 拇指第一关节旋转角
]

# --- 右手映射 ---

GLOVE_TO_R6 = [
    "r2",   # → L6[0] 大拇指弯曲   ← 拇指第一关节俯仰角
    "r20",  # → L6[1] 拇指旋转角   ← 拇指第一关节偏航角
    "r6",   # → L6[2] 食指弯曲     ← 食指第一关节俯仰角
    "r10",  # → L6[3] 中指弯曲     ← 中指第一关节俯仰角
    "r14",  # → L6[4] 无名指弯曲   ← 无名指第一关节俯仰角
    "r18",  # → L6[5] 小拇指弯曲   ← 小指第一关节俯仰角
]

GLOVE_TO_R7 = [
    "r2",   # → L7[0] 大拇指弯曲   ← 拇指第一关节俯仰角
    "r20",  # → L6/7[1] 大拇指横摆   ← 拇指第一关节偏航角
    "r6",   # → L7[2] 食指弯曲     ← 食指第一关节俯仰角
    "r10",  # → L7[3] 中指弯曲     ← 中指第一关节俯仰角
    "r14",  # → L7[4] 无名指弯曲   ← 无名指第一关节俯仰角
    "r18",  # → L7[5] 小拇指弯曲   ← 小指第一关节俯仰角
    "r20",  # → L7[6] 拇指旋转     ← 拇指第一关节旋转角
]

GLOVE_TO_R10 = [
    "r2",   # → L10[0] 拇指根部     ← 拇指第一关节俯仰角
    "r3",   # → L10[1] 拇指侧摆     ← 拇指第一关节偏航角
    "r6",   # → L10[2] 食指根部     ← 食指第一关节俯仰角
    "r10",  # → L10[3] 中指根部     ← 中指第一关节俯仰角
    "r14",  # → L10[4] 无名指根部   ← 无名指第一关节俯仰角
    "r18",  # → L10[5] 小指根部     ← 小指第一关节俯仰角
    "r7",   # → L10[6] 食指侧摆     ← 食指第一关节偏航角
    "r15",  # → L10[7] 无名指侧摆   ← 无名指第一关节偏航角
    "r19",  # → L10[8] 小指侧摆     ← 小指第一关节偏航角
    "r20",  # → L10[9] 拇指旋转     ← 拇指第一关节旋转角
]

# 汇总映射表
GLOVE_MAPPING = {
    "left":  {"L6": GLOVE_TO_L6, "O6": GLOVE_TO_L6, "L7": GLOVE_TO_L7, "L10": GLOVE_TO_L10},
    "right": {"L6": GLOVE_TO_R6, "O6": GLOVE_TO_R6, "L7": GLOVE_TO_R7, "L10": GLOVE_TO_R10},
}


class GloveToLinkerHand:
    """接收手套 UDP 数据并控制 LinkerHand"""

    def __init__(self, port=5555, log_dir="/home/elwg/Biolab/robot_data"):
        self.port = port
        self.sock = None
        self.running = False
        self.hands = {}  # {"left": {...}, "right": {...}}
        self.log_dir = log_dir
        self.hand_cmd_file = None
        self.hand_cmd_writer = None
        self.hand_cmd_header_written = False
        self.hand_cmd_flush_interval_ns = int(0.2 * 1e9)
        self.hand_cmd_last_flush_ns = time.time_ns()
        self.hand_cmd_rows_since_flush = 0

    def _init_hands(self):
        """根据 setting.yaml 初始化 LinkerHand API"""
        yaml = LoadWriteYaml()
        setting = yaml.load_setting_yaml()
        time.sleep(0.5)

        for side, key in [("left", "LEFT_HAND"), ("right", "RIGHT_HAND")]:
            cfg = setting["LINKER_HAND"][key]
            if not cfg["EXISTS"]:
                continue

            hand_joint = cfg["JOINT"]
            can = cfg["CAN"]
            modbus = cfg["MODBUS"]

            # 检查是否有对应的映射
            if hand_joint not in GLOVE_MAPPING.get(side, {}):
                print(f"[WARN] {side}手型号 {hand_joint} 暂不支持手套映射，跳过")
                continue

            print(f"[INFO] 初始化 {side}手: {hand_joint} CAN:{can}")
            try:
                api = LinkerHandApi(
                    hand_type=side,
                    hand_joint=hand_joint,
                    modbus=modbus,
                    can=can
                )
                self.hands[side] = {
                    "api": api,
                    "joint": hand_joint,
                    "mapping": GLOVE_MAPPING[side][hand_joint],
                }
                # 速度和力矩设置
                joint_len = {"L6": 6, "O6": 6, "L7": 7, "L10": 10}.get(hand_joint, 6)
                speed = 255
                api.set_speed([speed] * joint_len)
                torque = 100 if side == "left" else 255
                api.set_torque([torque] * joint_len)
                print(f"[OK] {side}手 ({hand_joint}) 速度={speed}, 力矩=255")
                print(f"[OK] {side}手 ({hand_joint}) 初始化成功")
            except Exception as e:
                print(f"[ERROR] {side}手初始化失败: {e}")

        if not self.hands:
            print("[ERROR] 没有任何手部设备初始化成功！")
            sys.exit(1)

    def start(self):
        """启动：初始化手部 + UDP 监听"""
        self._init_hands()

        # 初始化手部指令日志
        import os
        os.makedirs(self.log_dir, exist_ok=True)
        ts_str = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(self.log_dir, f"hand_commands_{ts_str}.csv")
        self.hand_cmd_file = open(log_path, 'w', newline='')
        self.hand_cmd_writer = csv.writer(self.hand_cmd_file)
        print(f"[INFO] 手部指令日志: {log_path}")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 缩小接收缓冲区，减少积压（默认可能几百KB，这里限制为64KB）
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(2)
        self.running = True

        signal.signal(signal.SIGINT, self._signal_handler)

        print(f"\n[INFO] 端口 {self.port} 监听中，等待手套数据... (Ctrl+C 退出)")
        print(f"[INFO] 已启用的手: {list(self.hands.keys())}")

        frame_count = 0
        while self.running:
            try:
                # 排空缓冲区，只保留最新一帧
                latest_data = None
                skipped = 0
                self.sock.setblocking(False)
                try:
                    while True:
                        data, addr = self.sock.recvfrom(65536)
                        if latest_data is not None:
                            skipped += 1
                        latest_data = data
                except BlockingIOError:
                    pass  # 缓冲区已空
                self.sock.setblocking(True)
                self.sock.settimeout(2)

                if latest_data is None:
                    time.sleep(0.005)
                    continue

                frame_count += 1
                if skipped > 0 and frame_count % 200 == 0:
                    print(f"[INFO] 帧 #{frame_count}: 跳过 {skipped} 帧旧数据")

                raw = latest_data.decode("utf-8", errors="replace")
                self._process(raw, frame_count)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[ERROR] 接收异常: {e}")

        self._shutdown()

    def _flush_hand_cmd_log(self, force=False):
        if not self.hand_cmd_file:
            return
        now_ns = time.time_ns()
        should_flush = force or (
            self.hand_cmd_rows_since_flush > 0 and
            now_ns - self.hand_cmd_last_flush_ns >= self.hand_cmd_flush_interval_ns
        )
        if should_flush:
            self.hand_cmd_file.flush()
            self.hand_cmd_last_flush_ns = now_ns
            self.hand_cmd_rows_since_flush = 0

    def _write_hand_command_row(self, side, positions):
        if not self.hand_cmd_writer:
            return
        try:
            sys_ts = time.time_ns()
            if not self.hand_cmd_header_written:
                hdr = ["system_timestamp_ns"]
                for s in ["left", "right"]:
                    for i in range(6):
                        hdr.append(f"{s}_hand_cmd{i+1}")
                self.hand_cmd_writer.writerow(hdr)
                self.hand_cmd_header_written = True

            row = [sys_ts]
            for s in ["left", "right"]:
                if s == side:
                    row.extend(positions)
                else:
                    row.extend([-1] * 6)
            self.hand_cmd_writer.writerow(row)
            self.hand_cmd_rows_since_flush += 1
            self._flush_hand_cmd_log()
        except Exception:
            pass

    def _process(self, raw: str, frame_count: int):
        """解析手套数据并驱动 LinkerHand"""
        try:
            value = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            print(f"[WARN] 帧 #{frame_count}: 非 JSON 数据")
            return

        param_dict = {}
        for role_name, device in value.items():
            parameters = device.get("Parameter", []) if isinstance(device, dict) else []
            if not parameters:
                continue

            for param in parameters:
                name = param.get("Name", "")
                val = param.get("Value", 0.0)
                param_dict[name] = val

        if not param_dict:
            return

        # 一帧UDP包里汇总所有参数后，每只手只驱动一次，避免重复发送旧值。
        for side, hand_info in self.hands.items():
            mapping = hand_info["mapping"]
            hand_joint = hand_info["joint"]
            api = hand_info["api"]
            side_prefix = "l" if side == "left" else "r"
            if not any(key.startswith(side_prefix) and key in param_dict for key in mapping):
                continue

            # 从手套数据中提取角度值（手套发送的是度数，负值表示弯曲）
            raw_degrees = [param_dict.get(key, 0.0) for key in mapping]

            # 度数 -> 弧度，取绝对值（手套弯曲为负值，SDK期望正值）
            arc_values = [abs(v) * math.pi / 180.0 for v in raw_degrees]

            # 弧度 -> 0~255 范围
            if side == "left":
                positions = arc_to_range_left(arc_values, hand_joint)
            else:
                positions = arc_to_range_right(arc_values, hand_joint)

            # 转为整数并限幅
            positions = [max(0, min(255, int(v))) for v in positions]

            # 右手拇指横摆固定255，食指固定255，后三指固定30
            if side == "right":
                if len(positions) > 1:
                    positions[1] = 255
                if len(positions) > 2:
                    positions[2] = 255
                for i in range(3, min(len(positions), 6)):
                    positions[i] = 30
                positions = clamp_thumb(positions, hand_joint)
            else:
                positions = remap_thumb_yaw(positions, side)
                positions = clamp_thumb(positions, hand_joint)

            try:
                api.finger_move(positions)
            except Exception as e:
                print(f"[ERROR] {side}手控制失败: {e}")
                continue

            self._write_hand_command_row(side, positions)

            if frame_count % 100 == 0:
                print(f"[帧 #{frame_count}] {side}手 ({hand_joint}): {positions}")

    def _signal_handler(self, sig, frame):
        print("\n[INFO] 收到退出信号，正在停止...")
        self.running = False

    def _shutdown(self):
        """清理退出"""
        for side, hand_info in self.hands.items():
            try:
                hand_info["api"].close_can()
                print(f"[INFO] {side}手 CAN 已关闭")
            except Exception:
                pass
        if self.hand_cmd_file:
            self._flush_hand_cmd_log(force=True)
            self.hand_cmd_file.close()
            print("[INFO] 手部指令日志已保存")
        if self.sock:
            self.sock.close()
        print("[INFO] 已退出。")


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5555
    controller = GloveToLinkerHand(port=port)
    controller.start()

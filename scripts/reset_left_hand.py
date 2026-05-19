#!/usr/bin/env python3
"""
左手重置到张开初始姿态
- 只连接左手 O6 (can3)，不影响右手
- 张开位置: [250, 250, 250, 250, 250, 250]
"""
import sys
import os
import time

sys.path.insert(0, '/home/elwg/Biolab/linkerhand-sdk')

from LinkerHand.linker_hand_api import LinkerHandApi

# O6 张开初始姿态
OPEN_POS = [250, 250, 250, 250, 250, 250]


def main():
    print("[1] 连接左手 O6 (can3) ...")
    hand = LinkerHandApi(hand_type="left", hand_joint="O6", modbus="None", can="can3")
    time.sleep(0.5)

    # 读取当前状态
    state = hand.get_state()
    print(f"[2] 当前左手状态: {state}")

    # 设置速度
    hand.set_speed([150] * 6)
    print(f"[3] 速度设为 150")

    # 移动到张开位置
    print(f"[4] 移动到张开位置: {OPEN_POS}")
    hand.finger_move(OPEN_POS)

    # 等待到位
    time.sleep(2)

    state = hand.get_state()
    print(f"[5] 到位后状态: {state}")

    # 关闭连接
    hand.close_can()
    print("[done] 左手已重置为张开姿态，连接已关闭")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
右手拇指横摆调到 255
- 只连接右手 O6 (can2)，不影响左手
"""
import sys
import os
import time

sys.path.insert(0, '/home/elwg/Biolab/linkerhand-sdk')

from LinkerHand.linker_hand_api import LinkerHandApi


def main():
    print("[1] 连接右手 O6 (can2) ...")
    hand = LinkerHandApi(hand_type="right", hand_joint="O6", modbus="None", can="can2")
    time.sleep(0.5)

    # 读取当前状态
    state = hand.get_state()
    print(f"[2] 当前右手状态: {state}")

    # 设置速度
    hand.set_speed([150] * 6)
    print(f"[3] 速度设为 150")

    # 只改拇指横摆(index 1) 为 255，其余保持当前状态
    target = list(state)
    target[1] = 255
    print(f"[4] 移动到: {target} (拇指横摆→255)")
    hand.finger_move(target)

    # 等待到位
    time.sleep(2)

    state = hand.get_state()
    print(f"[5] 到位后状态: {state}")

    # 关闭连接
    hand.close_can()
    print("[done] 右手拇指横摆已设为 255，连接已关闭")


if __name__ == "__main__":
    main()

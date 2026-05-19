#!/bin/bash
# Biolab 电机在线检测脚本
# 用法: bash scripts/check_motors.sh

source ~/miniconda3/etc/profile.d/conda.sh
conda activate /home/elwg/Biolab/conda_envs/ros_env

cd /tmp

python3 -c "
import openarm_can as oa
import time

for port in ['can0', 'can1']:
    print(f'=== {port} ===')
    try:
        arm = oa.OpenArm(port, True)
        arm.init_arm_motors(
            [oa.MotorType.DM8009, oa.MotorType.DM8009, oa.MotorType.DM4340, oa.MotorType.DM4340,
             oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310],
            [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
            [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
        )
        arm.set_callback_mode_all(oa.CallbackMode.STATE)
        arm.enable_all()
        time.sleep(0.5)
        arm.recv_all()
        motors = arm.get_arm().get_motors()
        ok = 0
        for i, m in enumerate(motors):
            pos = m.get_position()
            trq = m.get_torque()
            alive = abs(pos) > 0.0001 or abs(trq) > 0.0001
            if alive: ok += 1
            print(f'  电机{i+1}: pos={pos:.6f} torque={trq:.6f} {\"✅\" if alive else \"❌\"}')
        print(f'  => {ok}/7 在线')
        arm.disable_all()
    except Exception as e:
        print(f'  ❌ 错误: {e}')
    print()
"

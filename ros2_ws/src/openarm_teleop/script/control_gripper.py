#!/usr/bin/env python3
"""
OpenArm夹爪控制脚本
使用 GripperActionController 的 action 接口发送夹爪目标
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys


class GripperController(Node):
    def __init__(self, arm='left', position=0.04, effort=40.0, duration=2.0, steps=40):
        super().__init__('gripper_controller')

        self.arm = arm
        self.target_position = position
        self.effort = effort
        self.duration = duration
        self.steps = steps
        self.current_step = 0
        self.positions = []
        self.left_client = None
        self.right_client = None
        self.last_result = None

        # GripperActionController action 名称
        if arm == 'left':
            self.left_client = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
            self.get_logger().info(f'控制左臂夹爪，目标位置: {position}')
        elif arm == 'right':
            self.right_client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')
            self.get_logger().info(f'控制右臂夹爪，目标位置: {position}')
        elif arm == 'both':
            self.left_client = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
            self.right_client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')
            self.get_logger().info(f'控制双臂夹爪，目标位置: {position}')

        # 获取当前位置
        from sensor_msgs.msg import JointState
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.got_init_pos = False
        self.finished = False

    def joint_state_callback(self, msg):
        # 获取当前夹爪关节位置
        if self.got_init_pos:
            return
        left_name = 'openarm_left_finger_joint1'
        right_name = 'openarm_right_finger_joint1'
        left_pos = None
        right_pos = None
        if left_name in msg.name:
            idx = msg.name.index(left_name)
            left_pos = msg.position[idx]
        if right_name in msg.name:
            idx = msg.name.index(right_name)
            right_pos = msg.position[idx]
        if self.arm == 'left' and left_pos is not None:
            self.init_position = left_pos
            self.got_init_pos = True
        elif self.arm == 'right' and right_pos is not None:
            self.init_position = right_pos
            self.got_init_pos = True
        elif self.arm == 'both' and left_pos is not None and right_pos is not None:
            self.init_position = (left_pos, right_pos)
            self.got_init_pos = True
        if self.got_init_pos:
            self.get_logger().info(f'夹爪当前位置: {self.init_position}')
            self.start_interpolation()

    def start_interpolation(self):
        # 生成插值序列
        if self.arm == 'both':
            left_start, right_start = self.init_position
            left_seq = [left_start + (self.target_position - left_start) * i / self.steps for i in range(1, self.steps + 1)]
            right_seq = [right_start + (self.target_position - right_start) * i / self.steps for i in range(1, self.steps + 1)]
            self.positions = list(zip(left_seq, right_seq))
        else:
            start = self.init_position
            self.positions = [start + (self.target_position - start) * i / self.steps for i in range(1, self.steps + 1)]
        self.timer = self.create_timer(self.duration / self.steps, self.send_next_goal)
        self.get_logger().info(f'插值序列生成，步数: {self.steps}, 总时长: {self.duration}s')

    def send_next_goal(self):
        if self.current_step >= len(self.positions):
            self.get_logger().info('夹爪匀速过渡完成')
            self.finished = True
            rclpy.shutdown()
            import os
            os._exit(0)
            return
        if self.arm == 'both':
            left_pos, right_pos = self.positions[self.current_step]
            self.send_goal(self.left_client, left_pos, '左臂')
            self.send_goal(self.right_client, right_pos, '右臂')
        elif self.arm == 'left':
            self.send_goal(self.left_client, self.positions[self.current_step], '左臂')
        elif self.arm == 'right':
            self.send_goal(self.right_client, self.positions[self.current_step], '右臂')
        self.current_step += 1

    def send_goal(self, client, pos, side: str):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(pos)
        goal_msg.command.max_effort = float(self.effort)
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'等待 {side} 夹爪控制器超时')
            return None
        self.get_logger().info(f'发送{side}夹爪目标: 位置={pos}, 力={self.effort}')
        client.send_goal_async(goal_msg)


def main(args=None):
    if len(sys.argv) < 2:
        print("用法:")
        print("  左臂夹爪:   python3 control_gripper.py left <position> [max_effort] [duration] [steps]")
        print("  右臂夹爪:   python3 control_gripper.py right <position> [max_effort] [duration] [steps]")
        print("  双臂夹爪:   python3 control_gripper.py both <position> [max_effort] [duration] [steps]")
        print("\n参数说明:")
        print("  position: 0.0=完全闭合, 0.04=完全张开 (默认: 0.04)")
        print("  max_effort: 最大施加力矩/力 (默认: 40.0)")
        print("  duration: 总过渡时间(秒，默认: 1.0)")
        print("  steps: 插值步数(默认: 20)")
        print("\n示例:")
        print("  python3 control_gripper.py left 0.02 20 0.8 16   # 左臂夹爪半开，最大力20，0.8秒，16步")
        print("  python3 control_gripper.py both 0.0 40 1.0 20    # 双臂夹爪闭合，1秒，20步")
        sys.exit(1)

    arm = sys.argv[1]
    position = float(sys.argv[2]) if len(sys.argv) > 2 else 0.04
    effort = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0
    duration = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    steps = int(sys.argv[5]) if len(sys.argv) > 5 else 20

    if arm not in ['left', 'right', 'both']:
        print(f"错误: 无效的臂类型 '{arm}'")
        print("有效选项: left, right, both")
        sys.exit(1)

    rclpy.init(args=args)
    controller = GripperController(arm, position, effort, duration, steps)

    import time
    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            if getattr(controller, 'finished', False):
                break
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()


if __name__ == '__main__':
    main()

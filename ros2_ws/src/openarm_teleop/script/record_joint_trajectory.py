#!/usr/bin/env python3
"""
OpenArm关节轨迹录制节点
订阅关节状态话题，录制关节数据并保存到JSON文件

使用方法:
    ros2 run openarm_teleop record_joint_trajectory.py --output trajectory.json
    或
    python3 record_joint_trajectory.py --output trajectory.json
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import json
import argparse
import os
import signal
import sys
import shutil
from datetime import datetime
from collections import deque


class JointTrajectoryRecorder(Node):
    def __init__(self, output_file, record_commands=False, save_interval=1.0):
        super().__init__('joint_trajectory_recorder')
        
        self.output_file = output_file
        self.record_commands = record_commands
        self.recording = False
        self.trajectory_data = []
        self.start_time = None
        self.save_interval = save_interval
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 可选：订阅命令话题
        if record_commands:
            self.left_cmd_sub = self.create_subscription(
                Float64MultiArray,
                '/left_forward_position_controller/commands',
                self.left_command_callback,
                10
            )
            self.right_cmd_sub = self.create_subscription(
                Float64MultiArray,
                '/right_forward_position_controller/commands',
                self.right_command_callback,
                10
            )
            self.left_commands = deque(maxlen=1000)
            self.right_commands = deque(maxlen=1000)
        
        # 创建服务来开始/停止录制
        from std_srvs.srv import SetBool
        self.start_service = self.create_service(
            SetBool,
            'start_recording',
            self.start_recording_callback
        )
        self.stop_service = self.create_service(
            SetBool,
            'stop_recording',
            self.stop_recording_callback
        )
        
        # 定时器用于定期保存（防止数据丢失）
        self.save_timer = self.create_timer(self.save_interval, self.auto_save)
        self.last_save_count = 0  # 记录上次保存的数据点数量
        
        self.get_logger().info(f'✅ 关节轨迹录制节点已启动')
        self.get_logger().info(f'   - 输出文件: {output_file}')
        self.get_logger().info(f'   - 录制命令: {record_commands}')
        self.get_logger().info(f'   - 使用服务 /start_recording 开始录制')
        self.get_logger().info(f'   - 使用服务 /stop_recording 停止录制并保存')
        
        # 自动开始录制
        self.recording = True
        self.start_time = self.get_clock().now()
        self.get_logger().info('🎬 自动开始录制...')
    
    def joint_state_callback(self, msg: JointState):
        """处理关节状态消息"""
        if not self.recording:
            return
        
        # 计算相对时间戳（从开始录制算起）
        current_time = self.get_clock().now()
        if self.start_time is None:
            self.start_time = current_time
        
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # 转换为秒
        
        # 提取左右臂关节数据
        left_arm_joints = []
        right_arm_joints = []
        left_gripper_joints = []
        right_gripper_joints = []
        
        # 定义关节名称
        left_arm_names = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        right_arm_names = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        left_gripper_names = ['openarm_left_finger_joint1']
        right_gripper_names = ['openarm_right_finger_joint1']
        
        # 提取关节位置
        for i, name in enumerate(msg.name):
            if name in left_arm_names:
                idx = left_arm_names.index(name)
                while len(left_arm_joints) <= idx:
                    left_arm_joints.append(0.0)
                left_arm_joints[idx] = msg.position[i] if i < len(msg.position) else 0.0
            elif name in right_arm_names:
                idx = right_arm_names.index(name)
                while len(right_arm_joints) <= idx:
                    right_arm_joints.append(0.0)
                right_arm_joints[idx] = msg.position[i] if i < len(msg.position) else 0.0
            elif name in left_gripper_names:
                idx = left_gripper_names.index(name)
                while len(left_gripper_joints) <= idx:
                    left_gripper_joints.append(0.0)
                left_gripper_joints[idx] = msg.position[i] if i < len(msg.position) else 0.0
            elif name in right_gripper_names:
                idx = right_gripper_names.index(name)
                while len(right_gripper_joints) <= idx:
                    right_gripper_joints.append(0.0)
                right_gripper_joints[idx] = msg.position[i] if i < len(msg.position) else 0.0
        
        # 确保数组长度正确
        while len(left_arm_joints) < 7:
            left_arm_joints.append(0.0)
        while len(right_arm_joints) < 7:
            right_arm_joints.append(0.0)
        while len(left_gripper_joints) < 1:
            left_gripper_joints.append(0.0)
        while len(right_gripper_joints) < 1:
            right_gripper_joints.append(0.0)
        
        # 获取命令（如果启用）
        left_cmd = None
        right_cmd = None
        if self.record_commands and self.left_commands:
            left_cmd = list(self.left_commands[-1]) if self.left_commands else None
        if self.record_commands and self.right_commands:
            right_cmd = list(self.right_commands[-1]) if self.right_commands else None
        
        # 保存数据点
        data_point = {
            'time': elapsed_time,
            'left_arm': left_arm_joints[:7],
            'right_arm': right_arm_joints[:7],
            'left_gripper': left_gripper_joints[0] if left_gripper_joints else 0.0,
            'right_gripper': right_gripper_joints[0] if right_gripper_joints else 0.0,
        }
        
        if left_cmd is not None:
            data_point['left_command'] = left_cmd
        if right_cmd is not None:
            data_point['right_command'] = right_cmd
        
        self.trajectory_data.append(data_point)
    
    def left_command_callback(self, msg: Float64MultiArray):
        """处理左臂命令"""
        if self.recording:
            self.left_commands.append(list(msg.data))
    
    def right_command_callback(self, msg: Float64MultiArray):
        """处理右臂命令"""
        if self.recording:
            self.right_commands.append(list(msg.data))
    
    def start_recording_callback(self, _request, response):
        """开始录制服务回调"""
        if not self.recording:
            self.recording = True
            self.start_time = self.get_clock().now()
            self.trajectory_data = []
            self.get_logger().info('🎬 开始录制...')
            response.success = True
            response.message = 'Recording started'
        else:
            response.success = False
            response.message = 'Already recording'
        return response
    
    def stop_recording_callback(self, _request, response):
        """停止录制服务回调"""
        if self.recording:
            self.recording = False
            self.save_trajectory()
            response.success = True
            response.message = f'Recording stopped and saved to {self.output_file}'
        else:
            response.success = False
            response.message = 'Not recording'
        return response
    
    def auto_save(self):
        """自动保存（防止数据丢失）"""
        if self.recording and len(self.trajectory_data) > 0:
            # 只在有新数据时才保存（避免频繁写入）
            if len(self.trajectory_data) > self.last_save_count:
                self.save_trajectory(silent=True)  # silent模式不输出日志
                self.last_save_count = len(self.trajectory_data)
                # 每10秒输出一次保存信息
                if len(self.trajectory_data) % 100 == 0:  # 假设100Hz，即每1秒
                    self.get_logger().info(f'💾 自动保存: {len(self.trajectory_data)} 个数据点')
    
    def save_trajectory(self, silent=False):
        """保存轨迹到文件（使用原子写入确保数据完整性）"""
        if not self.trajectory_data:
            if not silent:
                self.get_logger().warn('⚠️  没有数据可保存')
            return
        
        # 创建输出目录（如果不存在）
        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 准备保存的数据
        trajectory = {
            'metadata': {
                'created_at': datetime.now().isoformat(),
                'total_points': len(self.trajectory_data),
                'duration': self.trajectory_data[-1]['time'] if self.trajectory_data else 0.0,
                'record_commands': self.record_commands,
            },
            'trajectory': self.trajectory_data
        }
        
        # 使用原子写入：先写入临时文件，再重命名（确保数据完整性）
        try:
            # 创建临时文件
            temp_file = self.output_file + '.tmp'
            with open(temp_file, 'w', encoding='utf-8') as f:
                json.dump(trajectory, f, indent=2)
            
            # 原子性重命名（在大多数文件系统上是原子操作）
            shutil.move(temp_file, self.output_file)
            
            if not silent:
                self.get_logger().info(
                    f'✅ 轨迹已保存: {self.output_file} '
                    f'({len(self.trajectory_data)} 个数据点, '
                    f'时长: {trajectory["metadata"]["duration"]:.2f}秒)'
                )
        except Exception as e:
            self.get_logger().error(f'❌ 保存失败: {e}')
            # 清理临时文件
            temp_file = self.output_file + '.tmp'
            if os.path.exists(temp_file):
                try:
                    os.remove(temp_file)
                except OSError:
                    pass


def setup_signal_handlers(recorder):
    """设置信号处理，确保退出时保存数据"""
    def signal_handler(signum, _frame):
        try:
            recorder.get_logger().info(f'🛑 收到信号 {signum}，停止录制并保存...')
            if recorder.recording:
                recorder.recording = False
                recorder.save_trajectory()
        except Exception as e:
            print(f'保存数据时出错: {e}')
        finally:
            try:
                recorder.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)
    
    # 注册信号处理（Windows可能不支持SIGTERM）
    try:
        signal.signal(signal.SIGINT, signal_handler)
    except (AttributeError, OSError):
        pass
    try:
        signal.signal(signal.SIGTERM, signal_handler)
    except (AttributeError, OSError):
        pass


def main(args=None):
    parser = argparse.ArgumentParser(description='录制OpenArm关节轨迹')
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=f'trajectory_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json',
        help='输出JSON文件路径'
    )
    parser.add_argument(
        '--record-commands',
        action='store_true',
        help='同时录制命令话题'
    )
    parser.add_argument(
        '--save-interval',
        type=float,
        default=1.0,
        help='自动保存间隔（秒），默认1.0秒'
    )
    
    args = parser.parse_args()
    
    rclpy.init(args=None)
    
    recorder = JointTrajectoryRecorder(
        output_file=args.output,
        record_commands=args.record_commands,
        save_interval=args.save_interval
    )
    
    # 设置信号处理（确保退出时保存）
    setup_signal_handlers(recorder)
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('🛑 收到中断信号，停止录制...')
        if recorder.recording:
            recorder.recording = False
            recorder.save_trajectory()
    except Exception as e:
        recorder.get_logger().error(f'❌ 发生错误: {e}')
        if recorder.recording:
            recorder.recording = False
            recorder.save_trajectory()
    finally:
        # 确保最终保存
        if recorder.recording:
            recorder.recording = False
            recorder.save_trajectory()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
OpenArm关节轨迹播放节点
读取JSON文件中的关节数据，发布命令到控制器话题

使用方法:
    ros2 run openarm_teleop play_joint_trajectory.py --input trajectory.json
    或
    python3 play_joint_trajectory.py --input trajectory.json
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import json
import argparse
import os
import time
from threading import Thread
from collections import deque


class JointTrajectoryPlayer(Node):
    def __init__(self, input_file, loop=False, speed_factor=1.0, start_time=0.0, max_frequency=100.0, settle_time=1.0):
        super().__init__('joint_trajectory_player')
        
        self.input_file = input_file
        self.loop = loop
        self.speed_factor = speed_factor
        self.start_time = start_time
        self.max_frequency = max_frequency  # 最大发布频率（Hz），匹配控制器频率
        self.min_interval = 1.0 / max_frequency if max_frequency > 0 else 0.01
        self.settle_time = settle_time  # 播放结束后保持最后一帧的时间（秒），让机械臂跟到目标角度
        self.trajectory_data = None
        self.metadata = None
        self.playing = False
        self.play_thread = None
        
        # 发布关节命令
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            10
        )
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
            10
        )
        
        # 夹爪Action客户端
        self.left_gripper_client = ActionClient(
            self, GripperCommand, '/left_gripper_controller/gripper_cmd'
        )
        self.right_gripper_client = ActionClient(
            self, GripperCommand, '/right_gripper_controller/gripper_cmd'
        )
        
        # 创建服务来控制播放
        from std_srvs.srv import SetBool
        self.start_service = self.create_service(
            SetBool,
            'start_playback',
            self.start_playback_callback
        )
        self.stop_service = self.create_service(
            SetBool,
            'stop_playback',
            self.stop_playback_callback
        )
        
        # 加载轨迹文件
        if not self.load_trajectory():
            self.get_logger().error('❌ 无法加载轨迹文件，节点将退出')
            return
        
        self.get_logger().info(f'✅ 关节轨迹播放节点已启动')
        self.get_logger().info(f'   - 输入文件: {input_file}')
        self.get_logger().info(f'   - 数据点: {self.metadata["total_points"]}')
        self.get_logger().info(f'   - 时长: {self.metadata["duration"]:.2f}秒')
        self.get_logger().info(f'   - 速度因子: {speed_factor}x')
        self.get_logger().info(f'   - 循环播放: {loop}')
        self.get_logger().info(f'   - 最大发布频率: {self.max_frequency}Hz（匹配控制器）')
        self.get_logger().info(f'   - 末端稳位时间: {self.settle_time}秒（播放结束后保持最后一帧）')
        self.get_logger().info(f'   - 播放模式: 精确时间还原（保持原始动作）')
        self.get_logger().info(f'   - 使用服务 /start_playback 开始播放')
        self.get_logger().info(f'   - 使用服务 /stop_playback 停止播放')
        
        # 等待夹爪Action服务器
        self.get_logger().info('⏳ 等待夹爪Action服务器...')
        if not self.left_gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('⚠️  左夹爪Action服务器未就绪')
        if not self.right_gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('⚠️  右夹爪Action服务器未就绪')
        
        # 自动开始播放
        self.start_playback()
    
    def load_trajectory(self):
        """加载轨迹文件"""
        if not os.path.exists(self.input_file):
            self.get_logger().error(f'❌ 文件不存在: {self.input_file}')
            return False
        
        try:
            with open(self.input_file, 'r') as f:
                data = json.load(f)
            
            self.metadata = data.get('metadata', {})
            self.trajectory_data = data.get('trajectory', [])
            
            if not self.trajectory_data:
                self.get_logger().error('❌ 轨迹文件为空')
                return False
            
            self.get_logger().info(f'✅ 成功加载轨迹: {len(self.trajectory_data)} 个数据点')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 加载轨迹文件失败: {e}')
            return False
    
    def start_playback_callback(self, request, response):
        """开始播放服务回调"""
        if not self.playing:
            self.start_playback()
            response.success = True
            response.message = 'Playback started'
        else:
            response.success = False
            response.message = 'Already playing'
        return response
    
    def stop_playback_callback(self, request, response):
        """停止播放服务回调"""
        if self.playing:
            self.playing = False
            if self.play_thread:
                self.play_thread.join(timeout=1.0)
            response.success = True
            response.message = 'Playback stopped'
        else:
            response.success = False
            response.message = 'Not playing'
        return response
    
    def start_playback(self):
        """开始播放"""
        if self.playing:
            self.get_logger().warn('⚠️  已经在播放中')
            return
        
        if not self.trajectory_data:
            self.get_logger().error('❌ 没有轨迹数据可播放')
            return
        
        self.playing = True
        self.play_thread = Thread(target=self.play_trajectory, daemon=True)
        self.play_thread.start()
        self.get_logger().info('🎬 开始播放轨迹...')
    
    def play_trajectory(self):
        """播放轨迹（在独立线程中运行，精确还原原始时间）"""
        if not self.trajectory_data:
            return
        
        # 找到起始点
        start_idx = 0
        if self.start_time > 0:
            for i, point in enumerate(self.trajectory_data):
                if point['time'] >= self.start_time:
                    start_idx = i
                    break
        
        last_gripper_left = None
        last_gripper_right = None
        gripper_threshold = 0.001  # 夹爪变化阈值
        
        try:
            while self.playing:
                # 记录播放开始时间
                playback_start_time = time.perf_counter()
                trajectory_start_time = self.trajectory_data[start_idx]['time'] if start_idx < len(self.trajectory_data) else 0.0
                
                # 从起始点开始播放
                for i in range(start_idx, len(self.trajectory_data)):
                    if not self.playing:
                        break
                    
                    point = self.trajectory_data[i]
                    
                    # 计算应该播放的时间点（基于原始时间戳）
                    expected_elapsed = (point['time'] - trajectory_start_time) / self.speed_factor
                    actual_elapsed = time.perf_counter() - playback_start_time
                    
                    # 如果实际时间落后，需要等待；如果超前，说明播放太快，需要调整
                    time_error = expected_elapsed - actual_elapsed
                    
                    # 发布左臂命令
                    if 'left_arm' in point and len(point['left_arm']) == 7:
                        left_msg = Float64MultiArray()
                        left_msg.data = point['left_arm']
                        self.left_arm_pub.publish(left_msg)
                    
                    # 发布右臂命令
                    if 'right_arm' in point and len(point['right_arm']) == 7:
                        right_msg = Float64MultiArray()
                        right_msg.data = point['right_arm']
                        self.right_arm_pub.publish(right_msg)
                    
                    # 发布夹爪命令（只在变化时发布）
                    if 'left_gripper' in point:
                        gripper_val = point['left_gripper']
                        if last_gripper_left is None or abs(gripper_val - last_gripper_left) > gripper_threshold:
                            self.send_gripper_command('left', gripper_val)
                            last_gripper_left = gripper_val
                    
                    if 'right_gripper' in point:
                        gripper_val = point['right_gripper']
                        if last_gripper_right is None or abs(gripper_val - last_gripper_right) > gripper_threshold:
                            self.send_gripper_command('right', gripper_val)
                            last_gripper_right = gripper_val
                    
                    # 计算到下一个点的时间间隔
                    if i < len(self.trajectory_data) - 1:
                        next_point = self.trajectory_data[i + 1]
                        dt = (next_point['time'] - point['time']) / self.speed_factor
                        # 限制最小间隔，避免超过控制器处理能力；允许更长间隔以保持时间线
                        dt = max(dt, self.min_interval)
                    else:
                        # 最后一个点，只等一个周期，末端稳位在播完后统一做
                        dt = self.min_interval
                    
                    # 精确等待到下一个时间点
                    # 考虑时间误差，如果已经落后，减少等待时间；如果超前，正常等待
                    if dt > 0:
                        if time_error > 0:
                            # 实际时间落后，需要等待，但可以稍微补偿
                            sleep_time = max(self.min_interval, dt - time_error * 0.5)  # 部分补偿，但不小于最小间隔
                        else:
                            # 实际时间超前或正常，正常等待
                            sleep_time = dt
                        
                        if sleep_time > 0:
                            # 使用高精度sleep
                            if sleep_time > 0.001:  # 大于1ms时使用普通sleep
                                time.sleep(sleep_time)
                            else:  # 小于1ms时使用忙等待（更精确）
                                end_time = time.perf_counter() + sleep_time
                                while time.perf_counter() < end_time:
                                    pass
                
                # 非循环时：保持最后一帧一段时间，让机械臂跟到目标角度（提高回放精度）
                if not self.loop and self.playing and self.settle_time > 0 and len(self.trajectory_data) > 0:
                    last_point = self.trajectory_data[-1]
                    settle_end = time.perf_counter() + self.settle_time
                    self.get_logger().info(f'⏳ 末端稳位 {self.settle_time}秒，保持最后一帧...')
                    while self.playing and time.perf_counter() < settle_end:
                        if 'left_arm' in last_point and len(last_point['left_arm']) == 7:
                            left_msg = Float64MultiArray()
                            left_msg.data = last_point['left_arm']
                            self.left_arm_pub.publish(left_msg)
                        if 'right_arm' in last_point and len(last_point['right_arm']) == 7:
                            right_msg = Float64MultiArray()
                            right_msg.data = last_point['right_arm']
                            self.right_arm_pub.publish(right_msg)
                        time.sleep(self.min_interval)
                
                # 如果循环播放，重新开始
                if self.loop and self.playing:
                    start_idx = 0
                    self.get_logger().info('🔄 循环播放，重新开始...')
                else:
                    break
            
            self.get_logger().info('✅ 播放完成')
            self.playing = False
            
        except Exception as e:
            self.get_logger().error(f'❌ 播放过程中出错: {e}')
            self.playing = False
    
    def send_gripper_command(self, side, position):
        """发送夹爪命令"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 40.0
        
        if side == 'left' and self.left_gripper_client.server_is_ready():
            self.left_gripper_client.send_goal_async(goal_msg)
        elif side == 'right' and self.right_gripper_client.server_is_ready():
            self.right_gripper_client.send_goal_async(goal_msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='播放OpenArm关节轨迹')
    parser.add_argument(
        '--input', '-i',
        type=str,
        required=True,
        help='输入JSON文件路径'
    )
    parser.add_argument(
        '--loop', '-l',
        action='store_true',
        help='循环播放'
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=1.0,
        help='播放速度因子（1.0=正常速度，2.0=2倍速）'
    )
    parser.add_argument(
        '--start-time',
        type=float,
        default=0.0,
        help='从指定时间（秒）开始播放'
    )
    parser.add_argument(
        '--max-frequency',
        type=float,
        default=100.0,
        help='最大发布频率（Hz），默认100Hz匹配控制器频率'
    )
    parser.add_argument(
        '--settle-time',
        type=float,
        default=1.0,
        help='播放结束后保持最后一帧的时间（秒），让机械臂跟到目标角度，默认1.0。若回放角度略小可增大到1.5或2.0'
    )
    
    args = parser.parse_args()
    
    rclpy.init(args=None)
    
    player = JointTrajectoryPlayer(
        input_file=args.input,
        loop=args.loop,
        speed_factor=args.speed,
        start_time=args.start_time,
        max_frequency=args.max_frequency,
        settle_time=args.settle_time
    )
    
    try:
        rclpy.spin(player)
    except KeyboardInterrupt:
        player.get_logger().info('🛑 收到中断信号，停止播放...')
        player.playing = False
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
OpenArm末端执行器位置控制
直接指定末端位置(x,y,z)和姿态，整个机械臂（所有关节）会自动规划路径并运动到该位置

工作原理：
- 使用MoveIt2的move_group action进行运动规划
- MoveIt2计算逆运动学（IK），确定所有7个关节的目标角度
- 进行路径规划，生成平滑的关节轨迹
- 执行运动，整个机械臂协调运动，使末端执行器到达目标位置

前置条件:
1. 安装MoveIt2: sudo apt install ros-humble-moveit
2. 启动: ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.action import MoveGroup
from std_msgs.msg import Header


def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """欧拉角转四元数"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy
    )


class MoveToPoseNode(Node):
    def __init__(self, arm_side, x, y, z, roll, pitch, yaw):
        super().__init__('move_to_pose_node')
        
        self.arm_side = arm_side
        self.planning_group = f"{arm_side}_arm"  # left_arm 或 right_arm
        self.result_received = False
        self.success = False
        
        # MoveIt2 move_action action client (使用MoveGroup类型，但连接到/move_action)
        action_name = '/move_action'
        self.move_group_client = ActionClient(self, MoveGroup, action_name)
        
        # 等待action server
        self.get_logger().info(f'等待MoveIt2 action server: {action_name}')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'无法连接到MoveIt2 action server: {action_name}')
            self.get_logger().error('请确保MoveIt2已启动: ros2 launch openarm_bimanual_moveit_config demo.launch.py')
            self.get_logger().info('提示: 可以运行 ros2 action list 查看可用的action')
            return
        
        # 创建目标pose
        target_pose = PoseStamped()
        target_pose.header = Header()
        target_pose.header.frame_id = 'openarm_link0'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position = Point(x=x, y=y, z=z)
        target_pose.pose.orientation = euler_to_quaternion(roll, pitch, yaw)
        
        print(f"目标位置: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}")
        print(f"规划组: {self.planning_group}")
        print(f"末端执行器: openarm_{arm_side}_hand")
        print("提示: 如果失败，尝试更接近当前位置的目标，或在RViz中查看工作空间")
        
        # 创建action goal
        goal_msg = MoveGroup.Goal()
        
        # 设置规划组
        goal_msg.request.group_name = self.planning_group
        
        # 设置工作空间参数
        goal_msg.request.workspace_parameters.header = target_pose.header
        goal_msg.request.workspace_parameters.header.frame_id = 'openarm_link0'
        # 设置工作空间边界（可选，让MoveIt2自动计算）
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.min_corner.z = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.0
        
        # 添加位置和姿态约束 - 使用更简单的方式
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        
        constraints = Constraints()
        
        # 位置约束 - 使用更大的容差区域
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = f'openarm_{arm_side}_hand'
        from shape_msgs.msg import SolidPrimitive
        # 使用球形容差区域，增大容差
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.15]  # 15cm容差，给MoveIt2更多灵活性
        pos_constraint.constraint_region.primitives = [sphere]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        pos_constraint.weight = 1.0
        constraints.position_constraints = [pos_constraint]
        
        # 姿态约束 - 允许更大的容差（可选，如果位置约束足够可以注释掉）
        # 对于7自由度机械臂，通常只需要位置约束即可
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = f'openarm_{arm_side}_hand'
        ori_constraint.orientation = target_pose.pose.orientation
        # 使用非常大的姿态容差，让MoveIt2自由选择姿态
        ori_constraint.absolute_x_axis_tolerance = 3.14  # 约180度
        ori_constraint.absolute_y_axis_tolerance = 3.14
        ori_constraint.absolute_z_axis_tolerance = 3.14
        ori_constraint.weight = 0.1  # 降低权重，优先满足位置约束
        constraints.orientation_constraints = [ori_constraint]
        
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 20.0
        
        # 规划选项
        goal_msg.planning_options.plan_only = False  # False表示规划并执行，True表示只规划
        goal_msg.planning_options.look_around = True  # 允许在目标周围寻找可行解
        goal_msg.planning_options.look_around_attempts = 5  # 尝试次数
        
        # 发送goal
        self.get_logger().info('发送目标到MoveIt2...')
        self.send_goal_future = self.move_group_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            self.result_received = True
            self.success = False
            return
        
        self.get_logger().info('目标已接受，等待执行结果...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('✅ 已到达目标位置')
            self.success = True
        else:
            error_names = {
                1: 'SUCCESS',
                99999: 'FAILURE',
                -1: 'PLANNING_FAILED',
                -2: 'INVALID_MOTION_PLAN',
                -3: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                -4: 'CONTROL_FAILED',
                -5: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
                -6: 'TIMED_OUT',
                -7: 'PREEMPTED'
            }
            error_name = error_names.get(result.error_code.val, f'UNKNOWN({result.error_code.val})')
            self.get_logger().error(f'❌ 执行失败，错误代码: {error_name} (val={result.error_code.val})')
            self.get_logger().error('可能的原因:')
            self.get_logger().error('  1. 目标位置超出工作空间')
            self.get_logger().error('  2. 无法找到可行的路径')
            self.get_logger().error('  3. 目标位置与当前状态冲突')
            self.get_logger().info('提示: 尝试更接近当前位置的目标，或检查RViz中的工作空间')
            self.success = False
        self.result_received = True
    
    def feedback_callback(self, feedback_msg):
        # 可以在这里处理反馈信息
        pass


def main():
    if len(sys.argv) < 5:
        print("用法: python3 move_to_pose.py <left|right> <x> <y> <z> [--yaw <角度>]")
        print("示例: python3 move_to_pose.py left 0.3 0.2 0.4 --yaw 1.57")
        sys.exit(1)
    
    arm_side = sys.argv[1]
    x, y, z = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    
    roll = pitch = yaw = 0.0
    i = 5
    while i < len(sys.argv):
        if sys.argv[i] == '--roll' and i + 1 < len(sys.argv):
            roll = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--pitch' and i + 1 < len(sys.argv):
            pitch = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--yaw' and i + 1 < len(sys.argv):
            yaw = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1
    
    if arm_side not in ['left', 'right']:
        print("错误: arm_side 必须是 left 或 right")
        sys.exit(1)
    
    rclpy.init()
    node = MoveToPoseNode(arm_side, x, y, z, roll, pitch, yaw)
    
    # 等待结果
    import time
    timeout = 30.0
    start_time = time.time()
    while rclpy.ok() and not node.result_received:
        if time.time() - start_time > timeout:
            print("❌ 超时")
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(0 if node.success else 1)


if __name__ == '__main__':
    main()


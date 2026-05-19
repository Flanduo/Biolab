#!/usr/bin/env python3
"""
外骨骼命令桥接节点（参考 f710_teleop 的发布接口）

订阅 qnbot_teleoperator 的 /exo/gamepad_keys（及可选 /exo/joint_command），
发布与 f710_teleop 相同语义的话题，便于底盘/升降使用同一套接口。

发布话题（与 f710_teleop 一致）：
- /svtrobot_cmd (geometry_msgs/Twist)：底盘速度
  - linear.x：左摇杆前后
  - linear.y：左摇杆左右
  - angular.z：右摇杆左右
- /lift_control_cmd (std_msgs/Int32MultiArray)：升降 [direction, speed]
  - direction: 1=上升, -1=下降, 0=停止

可选：将 /exo/joint_command 转发到指定话题（默认不转发）。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy, JointState


class ExoCmdBridgeNode(Node):
    """将 /exo/gamepad_keys 转为 /svtrobot_cmd 与 /lift_control_cmd。"""

    def __init__(self):
        super().__init__("exo_cmd_bridge_node")

        # ---------- 订阅话题（qnbot_teleoperator 输出）----------
        self.declare_parameter("exo_gamepad_topic", "/exo/gamepad_keys")
        self.declare_parameter("exo_joint_command_topic", "/exo/joint_command")
        self.declare_parameter("forward_joint_command", False)
        self.declare_parameter("joint_command_forward_topic", "/exo/joint_command_forwarded")

        # ---------- 发布话题（与 f710_teleop 一致）----------
        self.declare_parameter("cmd_vel_topic", "/svtrobot_cmd")
        self.declare_parameter("lift_cmd_topic", "/lift_control_cmd")

        # ---------- 速度与缩放（对齐 f710_teleop 语义）----------
        self.declare_parameter("velocity.max_linear", 1.0)
        self.declare_parameter("velocity.max_angular", 1.0)
        self.declare_parameter("velocity.angular_scale", 1.0)
        # 方向取反（针对输入轴本身）
        self.declare_parameter("invert.left_x", False)
        self.declare_parameter("invert.left_y", False)
        self.declare_parameter("invert.right_x", False)
        self.declare_parameter("invert.right_y", False)
        self.declare_parameter("disable_left_x_when_right_active", False)
        self.declare_parameter("deadzone", 0.1)
        # Joy.axes 索引映射（不同数据源可能顺序不同）
        self.declare_parameter("axis.left_x_index", 0)
        self.declare_parameter("axis.left_y_index", 1)
        self.declare_parameter("axis.right_x_index", 2)
        self.declare_parameter("axis.right_y_index", 3)
        # 输入曲线：>1 减小小幅度灵敏度（sign(x)*|x|^expo）
        self.declare_parameter("input.expo_linear", 1.0)
        self.declare_parameter("input.expo_angular", 1.0)
        # 对角锁定：true=主轴优先，轻微偏转不会同时输出 vx/vy
        self.declare_parameter("input.axis_priority", True)
        # 次轴保留阈值：当 |minor| < |major| * ratio 时，minor 置 0（ratio 越大越“只走直线”）
        self.declare_parameter("input.axis_priority_ratio", 0.5)

        # ---------- 门控与调试 ----------
        # true=要求 /exo/gamepad_keys 的 buttons[3]==1 才发布底盘/升降；false=忽略该门控
        self.declare_parameter("require_vehicle_enabled", True)
        # true=周期性打印收到的 axes/buttons（用于排查“为什么一直是0/不触发”）
        self.declare_parameter("debug.print_joy", True)
        self.declare_parameter("debug.print_period_s", 1.0)
        # 若一直收不到 /exo/gamepad_keys，周期性提示（秒）
        self.declare_parameter("debug.no_msg_warn_period_s", 2.0)

        # ---------- 升降 ----------
        self.declare_parameter("lift.speed", 500)
        # Joy.buttons 索引：0=arms_control, 1=joystick_control, 2=homing, 3=vehicle_control
        # 4-9 左手柄: joystick_k,A,B,C,D,Switch  10-15 右手柄
        self.declare_parameter("button.lift_up", 5)   # 左手柄 A
        self.declare_parameter("button.lift_down", 6)  # 左手柄 B
        self.declare_parameter("button.emergency", 12)  # 右手柄 B
        # Joy.buttons 的按下电平：true=0 表示按下（active-low），false=1 表示按下
        self.declare_parameter("buttons.active_low", False)

        # ---------- 发布策略 ----------
        self.declare_parameter("cmd_vel.publish_always", False)

        exo_gamepad = self.get_parameter("exo_gamepad_topic").value
        exo_joint = self.get_parameter("exo_joint_command_topic").value
        forward_joint = self.get_parameter("forward_joint_command").value
        joint_forward_topic = self.get_parameter("joint_command_forward_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        lift_topic = self.get_parameter("lift_cmd_topic").value

        self.max_linear = float(self.get_parameter("velocity.max_linear").value)
        self.max_angular = float(self.get_parameter("velocity.max_angular").value)
        self.angular_scale = float(self.get_parameter("velocity.angular_scale").value)
        self.invert_left_x = bool(self.get_parameter("invert.left_x").value)
        self.invert_left_y = bool(self.get_parameter("invert.left_y").value)
        self.invert_right_x = bool(self.get_parameter("invert.right_x").value)
        self.invert_right_y = bool(self.get_parameter("invert.right_y").value)
        self.disable_left_x_when_right_active = bool(
            self.get_parameter("disable_left_x_when_right_active").value
        )
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.axis_left_x_index = int(self.get_parameter("axis.left_x_index").value)
        self.axis_left_y_index = int(self.get_parameter("axis.left_y_index").value)
        self.axis_right_x_index = int(self.get_parameter("axis.right_x_index").value)
        self.axis_right_y_index = int(self.get_parameter("axis.right_y_index").value)
        self.expo_linear = float(self.get_parameter("input.expo_linear").value)
        self.expo_angular = float(self.get_parameter("input.expo_angular").value)
        self.axis_priority = bool(self.get_parameter("input.axis_priority").value)
        self.axis_priority_ratio = float(
            self.get_parameter("input.axis_priority_ratio").value
        )
        self.lift_speed = int(self.get_parameter("lift.speed").value)
        self.btn_lift_up = int(self.get_parameter("button.lift_up").value)
        self.btn_lift_down = int(self.get_parameter("button.lift_down").value)
        self.btn_emergency = int(self.get_parameter("button.emergency").value)
        self.buttons_active_low = bool(self.get_parameter("buttons.active_low").value)
        self.cmd_vel_publish_always = bool(
            self.get_parameter("cmd_vel.publish_always").value
        )
        self.require_vehicle_enabled = bool(
            self.get_parameter("require_vehicle_enabled").value
        )
        self.debug_print_joy = bool(self.get_parameter("debug.print_joy").value)
        self.debug_print_period_s = float(
            self.get_parameter("debug.print_period_s").value
        )
        self.debug_no_msg_warn_period_s = float(
            self.get_parameter("debug.no_msg_warn_period_s").value
        )

        # 订阅 QoS：用 BEST_EFFORT 以兼容更多发布者配置
        self._sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(Int32MultiArray, lift_topic, 10)

        self._joy_sub = self.create_subscription(
            Joy,
            exo_gamepad,
            self._on_joy,
            self._sub_qos,
        )

        self._last_lift_direction = 0
        self._last_lift_speed = 0
        self._prev_cmd_vel_active = False
        self._last_debug_log_time = self.get_clock().now()
        self._joy_received_count = 0
        self._last_joy_received_time = None
        self._prev_emergency_pressed = False

        self._no_msg_timer = self.create_timer(
            max(0.2, self.debug_no_msg_warn_period_s),
            self._warn_if_no_joy,
        )

        self._joint_forward_pub = None
        if forward_joint:
            self._joint_forward_pub = self.create_publisher(
                JointState, joint_forward_topic, 10
            )
            self._joint_sub = self.create_subscription(
                JointState,
                exo_joint,
                self._on_joint_command,
                self._sub_qos,
            )

        self.get_logger().info(
            f"exo_cmd_bridge 已启动: 订阅 {exo_gamepad} -> 发布 {cmd_vel_topic}, {lift_topic}"
        )

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    @staticmethod
    def _apply_expo(value: float, expo: float) -> float:
        if expo <= 0.0:
            return value
        if expo == 1.0:
            return value
        sign = -1.0 if value < 0.0 else 1.0
        return sign * (abs(value) ** expo)

    def _apply_axis_priority(self, x: float, y: float) -> tuple[float, float]:
        """主轴优先：轻微对角时只保留主轴，避免“碰一点就同时发 vx+vy”."""
        if not self.axis_priority:
            return x, y
        ax, ay = abs(x), abs(y)
        if ax == 0.0 and ay == 0.0:
            return 0.0, 0.0
        if ax >= ay:
            # x 为主轴；若 y 太小则丢弃
            if ay < ax * self.axis_priority_ratio:
                y = 0.0
        else:
            # y 为主轴；若 x 太小则丢弃
            if ax < ay * self.axis_priority_ratio:
                x = 0.0
        return x, y

    def _on_joy(self, msg: Joy):
        self._joy_received_count += 1
        self._last_joy_received_time = self.get_clock().now()
        # 低频调试：确认是否真的收到 Joy，以及 axes/buttons 的值
        if self.debug_print_joy:
            now = self.get_clock().now()
            if (now - self._last_debug_log_time).nanoseconds >= int(
                self.debug_print_period_s * 1e9
            ):
                axes = list(msg.axes[:4]) if msg.axes else []
                buttons = list(msg.buttons[:16]) if msg.buttons else []
                self.get_logger().info(
                    f"recv /exo/gamepad_keys axes={axes} buttons16={buttons}"
                )
                self._last_debug_log_time = now

        if len(msg.buttons) <= max(self.btn_lift_up, self.btn_lift_down, self.btn_emergency):
            return

        def is_pressed(raw_value: int) -> bool:
            return (raw_value == 0) if self.buttons_active_low else (raw_value == 1)

        # 急停：右手柄 B（只在按下沿触发，避免刷屏）
        emergency_raw = int(msg.buttons[self.btn_emergency])
        emergency_pressed = is_pressed(emergency_raw)
        if emergency_pressed and not self._prev_emergency_pressed:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            lift_msg = Int32MultiArray()
            lift_msg.data = [0, 0]
            self.lift_pub.publish(lift_msg)
            self._last_lift_direction = 0
            self._last_lift_speed = 0
            self.get_logger().warn(
                f"急停(右手柄B)触发，已清零底盘与升降。emergency_raw={emergency_raw} active_low={self.buttons_active_low}"
            )
            self._prev_emergency_pressed = emergency_pressed
            return
        self._prev_emergency_pressed = emergency_pressed

        # 仅当 gamepad 中“小车/滑台控制”启用时才发布底盘与升降（buttons[3]）
        vehicle_enabled = len(msg.buttons) > 3 and msg.buttons[3] == 1
        if self.require_vehicle_enabled and not vehicle_enabled:
            return

        def get_axis(i: int) -> float:
            if 0 <= i < len(msg.axes):
                return float(msg.axes[i])
            return 0.0

        left_x = self._apply_deadzone(get_axis(self.axis_left_x_index))
        left_y = self._apply_deadzone(get_axis(self.axis_left_y_index))
        right_x = self._apply_deadzone(get_axis(self.axis_right_x_index))
        right_y = self._apply_deadzone(get_axis(self.axis_right_y_index))

        # 方向取反（按输入轴）
        if self.invert_left_x:
            left_x = -left_x
        if self.invert_left_y:
            left_y = -left_y
        if self.invert_right_x:
            right_x = -right_x
        if self.invert_right_y:
            right_y = -right_y

        # 对角锁定（只处理左摇杆的 xy）
        left_x, left_y = self._apply_axis_priority(left_x, left_y)

        # expo 曲线（减小小幅度灵敏度）
        left_x = self._apply_expo(left_x, self.expo_linear)
        left_y = self._apply_expo(left_y, self.expo_linear)
        right_x = self._apply_expo(right_x, self.expo_angular)

        vx = -left_y * self.max_linear
        vy = left_x * self.max_linear
        wz = right_x * self.max_angular * self.angular_scale

        if self.disable_left_x_when_right_active and abs(right_x) > 0:
            vy = 0.0

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self._sanitize_twist(twist)

        idle = self._twist_is_idle(twist)
        if self.cmd_vel_publish_always:
            self.cmd_vel_pub.publish(twist)
            self._prev_cmd_vel_active = not idle
        else:
            if not idle:
                self.cmd_vel_pub.publish(twist)
                self._prev_cmd_vel_active = True
            elif self._prev_cmd_vel_active:
                self.cmd_vel_pub.publish(twist)
                self._prev_cmd_vel_active = False

        # 升降：左手柄 A=上，B=下
        lift_up = is_pressed(int(msg.buttons[self.btn_lift_up]))
        lift_down = is_pressed(int(msg.buttons[self.btn_lift_down]))
        if lift_up and not lift_down:
            direction, speed = 1, self.lift_speed
        elif lift_down and not lift_up:
            direction, speed = -1, self.lift_speed
        else:
            direction, speed = 0, 0

        if direction != self._last_lift_direction or speed != self._last_lift_speed:
            lift_msg = Int32MultiArray()
            lift_msg.data = [direction, speed]
            self.lift_pub.publish(lift_msg)
            self._last_lift_direction = direction
            self._last_lift_speed = speed

    def _twist_is_idle(self, twist: Twist) -> bool:
        eps = 1e-9
        return (
            abs(twist.linear.x) < eps
            and abs(twist.linear.y) < eps
            and abs(twist.linear.z) < eps
            and abs(twist.angular.x) < eps
            and abs(twist.angular.y) < eps
            and abs(twist.angular.z) < eps
        )

    def _sanitize_twist(self, twist: Twist) -> None:
        eps = 1e-9
        for attr in ("x", "y", "z"):
            v = getattr(twist.linear, attr)
            setattr(twist.linear, attr, 0.0 if abs(v) < eps else float(v))
            v = getattr(twist.angular, attr)
            setattr(twist.angular, attr, 0.0 if abs(v) < eps else float(v))

    def _on_joint_command(self, msg: JointState):
        if self._joint_forward_pub is not None:
            self._joint_forward_pub.publish(msg)

    def _warn_if_no_joy(self) -> None:
        if not self.debug_print_joy:
            return
        if self._joy_received_count > 0:
            return
        self.get_logger().warn(
            "还没有收到 /exo/gamepad_keys。请确认 qnbot_teleoperator 正在发布该话题，且 ROS_DOMAIN_ID/网络一致。"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExoCmdBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

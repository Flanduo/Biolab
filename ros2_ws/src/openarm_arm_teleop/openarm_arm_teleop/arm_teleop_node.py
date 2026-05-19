import struct
import threading
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class LinuxJoystickReader(threading.Thread):
    """Simple Linux /dev/input/jsX reader thread."""

    def __init__(self, device_path: str, axes_count: int = 8, buttons_count: int = 12):
        super().__init__(daemon=True)
        self.device_path = device_path
        self.axes: List[float] = [0.0] * axes_count
        self.buttons: List[int] = [0] * buttons_count
        self._stop_event = threading.Event()
        self._device_fd = None

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self._device_fd is None:
                try:
                    self._device_fd = open(self.device_path, "rb", buffering=0)
                except OSError:
                    time.sleep(1.0)
                    continue

            try:
                data = self._device_fd.read(8)
                if not data or len(data) < 8:
                    self._device_fd.close()
                    self._device_fd = None
                    time.sleep(0.5)
                    continue

                _, value, etype, number = struct.unpack("IhBB", data)

                if etype & JS_EVENT_INIT:
                    etype &= ~JS_EVENT_INIT

                if etype == JS_EVENT_AXIS and 0 <= number < len(self.axes):
                    self.axes[number] = float(value) / 32767.0
                elif etype == JS_EVENT_BUTTON and 0 <= number < len(self.buttons):
                    self.buttons[number] = 1 if value else 0
            except OSError:
                if self._device_fd is not None:
                    try:
                        self._device_fd.close()
                    except OSError:
                        pass
                    self._device_fd = None
                time.sleep(0.5)

    def stop(self) -> None:
        self._stop_event.set()
        if self._device_fd is not None:
            try:
                self._device_fd.close()
            except OSError:
                pass
            self._device_fd = None


class OpenArmF710TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("openarm_f710_teleop")

        self.declare_parameter("device_path", "/dev/input/js0")
        self.declare_parameter("deadzone", 0.2)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("trajectory_dt", 0.2)
        self.declare_parameter("publish_on_input_only", True)
        self.declare_parameter("joint_speed", 0.8)  # rad/s
        self.declare_parameter("joint7_speed", 1.2)
        self.declare_parameter("gripper_step", 0.004)
        self.declare_parameter("gripper_min", 0.0)
        self.declare_parameter("gripper_max", 0.04)
        self.declare_parameter("gripper_effort", 40.0)

        self.declare_parameter("axis.left_x", 0)
        self.declare_parameter("axis.left_y", 1)
        self.declare_parameter("axis.right_x", 2)
        self.declare_parameter("axis.right_y", 3)
        self.declare_parameter("axis.hat_y", 5)

        self.declare_parameter("button.lb", 4)
        self.declare_parameter("button.rb", 5)
        self.declare_parameter("button.lt", 6)
        self.declare_parameter("button.rt", 7)
        self.declare_parameter("button.a", 1)
        self.declare_parameter("button.b", 2)
        self.declare_parameter("button.x", 0)
        self.declare_parameter("button.y", 3)

        self.declare_parameter("left_topic", "/left_joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("right_topic", "/right_joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("left_gripper_action", "/left_gripper_controller/gripper_cmd")
        self.declare_parameter("right_gripper_action", "/right_gripper_controller/gripper_cmd")

        self.declare_parameter("joint_limits.min", [-3.14] * 7)
        self.declare_parameter("joint_limits.max", [3.14] * 7)

        device_path = str(self.get_parameter("device_path").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.trajectory_dt = float(self.get_parameter("trajectory_dt").value)
        self.publish_on_input_only = bool(self.get_parameter("publish_on_input_only").value)
        self.joint_speed = float(self.get_parameter("joint_speed").value)
        self.joint7_speed = float(self.get_parameter("joint7_speed").value)
        self.gripper_step = float(self.get_parameter("gripper_step").value)
        self.gripper_min = float(self.get_parameter("gripper_min").value)
        self.gripper_max = float(self.get_parameter("gripper_max").value)
        self.gripper_effort = float(self.get_parameter("gripper_effort").value)

        self.axis_left_x = int(self.get_parameter("axis.left_x").value)
        self.axis_left_y = int(self.get_parameter("axis.left_y").value)
        self.axis_right_x = int(self.get_parameter("axis.right_x").value)
        self.axis_right_y = int(self.get_parameter("axis.right_y").value)
        self.axis_hat_y = int(self.get_parameter("axis.hat_y").value)

        self.btn_lb = int(self.get_parameter("button.lb").value)
        self.btn_rb = int(self.get_parameter("button.rb").value)
        self.btn_lt = int(self.get_parameter("button.lt").value)
        self.btn_rt = int(self.get_parameter("button.rt").value)
        self.btn_a = int(self.get_parameter("button.a").value)
        self.btn_b = int(self.get_parameter("button.b").value)
        self.btn_x = int(self.get_parameter("button.x").value)
        self.btn_y = int(self.get_parameter("button.y").value)

        self.left_topic = str(self.get_parameter("left_topic").value)
        self.right_topic = str(self.get_parameter("right_topic").value)

        self.left_joint_names = [f"openarm_left_joint{i}" for i in range(1, 8)]
        self.right_joint_names = [f"openarm_right_joint{i}" for i in range(1, 8)]
        self.left_gripper_name = "openarm_left_finger_joint1"
        self.right_gripper_name = "openarm_right_finger_joint1"

        min_limits = list(self.get_parameter("joint_limits.min").value)
        max_limits = list(self.get_parameter("joint_limits.max").value)
        if len(min_limits) != 7 or len(max_limits) != 7:
            self.get_logger().warn("joint_limits 长度必须为 7，已回退到默认 [-3.14, 3.14]")
            min_limits = [-3.14] * 7
            max_limits = [3.14] * 7
        self.min_limits = [float(v) for v in min_limits]
        self.max_limits = [float(v) for v in max_limits]

        self.left_pub = self.create_publisher(JointTrajectory, self.left_topic, 10)
        self.right_pub = self.create_publisher(JointTrajectory, self.right_topic, 10)

        self.left_gripper_client = ActionClient(
            self, GripperCommand, str(self.get_parameter("left_gripper_action").value)
        )
        self.right_gripper_client = ActionClient(
            self, GripperCommand, str(self.get_parameter("right_gripper_action").value)
        )

        self.joint_sub = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self.left_target: Optional[List[float]] = None
        self.right_target: Optional[List[float]] = None
        self.left_gripper_target = 0.0
        self.right_gripper_target = 0.0

        self.arm_mode = "both"  # left / right / both

        self._prev_a = 0
        self._prev_b = 0
        self._prev_x = 0
        self._prev_y = 0
        self._prev_hat_up = 0
        self._prev_hat_down = 0

        self.joy = LinuxJoystickReader(device_path=device_path)
        self.joy.start()

        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self.get_logger().info(
            f"openarm_arm_teleop started: device={device_path} mode={self.arm_mode}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_pos: Dict[str, float] = {}
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                name_to_pos[name] = msg.position[idx]

        if self.left_target is None and all(n in name_to_pos for n in self.left_joint_names):
            self.left_target = [name_to_pos[n] for n in self.left_joint_names]
            if self.left_gripper_name in name_to_pos:
                self.left_gripper_target = float(name_to_pos[self.left_gripper_name])

        if self.right_target is None and all(n in name_to_pos for n in self.right_joint_names):
            self.right_target = [name_to_pos[n] for n in self.right_joint_names]
            if self.right_gripper_name in name_to_pos:
                self.right_gripper_target = float(name_to_pos[self.right_gripper_name])

    def _axis(self, idx: int) -> float:
        if 0 <= idx < len(self.joy.axes):
            value = self.joy.axes[idx]
            return 0.0 if abs(value) < self.deadzone else value
        return 0.0

    def _button(self, idx: int) -> int:
        if 0 <= idx < len(self.joy.buttons):
            return self.joy.buttons[idx]
        return 0

    def _clamp(self, value: float, min_v: float, max_v: float) -> float:
        return max(min_v, min(max_v, value))

    def _send_gripper_goal(self, client: ActionClient, position: float) -> None:
        if not client.wait_for_server(timeout_sec=0.2):
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.gripper_effort)
        client.send_goal_async(goal)

    def _publish_traj(self, pub, names: List[str], positions: List[float]) -> None:
        msg = JointTrajectory()
        msg.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        sec = int(self.trajectory_dt)
        nsec = int((self.trajectory_dt - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec
        msg.points = [point]
        pub.publish(msg)

    def _handle_mode_switch(self) -> None:
        x = self._button(self.btn_x)
        y = self._button(self.btn_y)
        if x and not self._prev_x:
            self.arm_mode = "left"
            self.get_logger().info("arm mode -> left")
        if y and not self._prev_y:
            self.arm_mode = "right"
            self.get_logger().info("arm mode -> right")
        if x and y and (not self._prev_x or not self._prev_y):
            self.arm_mode = "both"
            self.get_logger().info("arm mode -> both")
        self._prev_x = x
        self._prev_y = y

    def _handle_gripper(self) -> None:
        hat_y = self._axis(self.axis_hat_y)
        up_pressed = 1 if hat_y > 0.5 else 0
        down_pressed = 1 if hat_y < -0.5 else 0

        if up_pressed and not self._prev_hat_up:
            if self.arm_mode in ("left", "both"):
                self.left_gripper_target = self._clamp(
                    self.left_gripper_target + self.gripper_step, self.gripper_min, self.gripper_max
                )
                self._send_gripper_goal(self.left_gripper_client, self.left_gripper_target)
            if self.arm_mode in ("right", "both"):
                self.right_gripper_target = self._clamp(
                    self.right_gripper_target + self.gripper_step, self.gripper_min, self.gripper_max
                )
                self._send_gripper_goal(self.right_gripper_client, self.right_gripper_target)

        if down_pressed and not self._prev_hat_down:
            if self.arm_mode in ("left", "both"):
                self.left_gripper_target = self._clamp(
                    self.left_gripper_target - self.gripper_step, self.gripper_min, self.gripper_max
                )
                self._send_gripper_goal(self.left_gripper_client, self.left_gripper_target)
            if self.arm_mode in ("right", "both"):
                self.right_gripper_target = self._clamp(
                    self.right_gripper_target - self.gripper_step, self.gripper_min, self.gripper_max
                )
                self._send_gripper_goal(self.right_gripper_client, self.right_gripper_target)

        self._prev_hat_up = up_pressed
        self._prev_hat_down = down_pressed

    def _joint_delta(self, dt: float) -> List[float]:
        # J1/J2/J3/J4 from sticks; J5/J6 from shoulder/trigger buttons; J7 from A/B.
        lx = self._axis(self.axis_left_x)
        ly = self._axis(self.axis_left_y)
        rx = self._axis(self.axis_right_x)
        ry = self._axis(self.axis_right_y)

        lb = self._button(self.btn_lb)
        rb = self._button(self.btn_rb)
        lt = self._button(self.btn_lt)
        rt = self._button(self.btn_rt)
        a = self._button(self.btn_a)
        b = self._button(self.btn_b)

        d = [0.0] * 7
        d[0] = lx * self.joint_speed * dt
        d[1] = -ly * self.joint_speed * dt
        d[2] = rx * self.joint_speed * dt
        d[3] = -ry * self.joint_speed * dt
        d[4] = ((1.0 if rb else 0.0) - (1.0 if lb else 0.0)) * self.joint_speed * dt
        d[5] = ((1.0 if rt else 0.0) - (1.0 if lt else 0.0)) * self.joint_speed * dt
        d[6] = ((1.0 if a else 0.0) - (1.0 if b else 0.0)) * self.joint7_speed * dt
        return d

    def _joint_input_active(self, delta: List[float]) -> bool:
        eps = 1e-9
        return any(abs(v) > eps for v in delta)

    def _on_timer(self) -> None:
        self._handle_mode_switch()
        self._handle_gripper()

        if self.left_target is None and self.right_target is None:
            return

        dt = 1.0 / self.publish_rate
        delta = self._joint_delta(dt)
        if self.publish_on_input_only and (not self._joint_input_active(delta)):
            return

        if self.arm_mode in ("left", "both") and self.left_target is not None:
            for i in range(7):
                self.left_target[i] = self._clamp(
                    self.left_target[i] + delta[i], self.min_limits[i], self.max_limits[i]
                )
            self._publish_traj(self.left_pub, self.left_joint_names, self.left_target)

        if self.arm_mode in ("right", "both") and self.right_target is not None:
            for i in range(7):
                self.right_target[i] = self._clamp(
                    self.right_target[i] + delta[i], self.min_limits[i], self.max_limits[i]
                )
            self._publish_traj(self.right_pub, self.right_joint_names, self.right_target)

    def destroy_node(self) -> bool:
        self.joy.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenArmF710TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

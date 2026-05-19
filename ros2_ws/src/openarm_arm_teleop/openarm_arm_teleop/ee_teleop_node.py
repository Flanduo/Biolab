import struct
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class LinuxJoystickReader(threading.Thread):
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


class OpenArmEETeleopNode(Node):
    """Joystick end-effector teleop using MoveIt IK service."""

    def __init__(self) -> None:
        super().__init__("openarm_ee_teleop")

        self.declare_parameter("device_path", "/dev/input/js1")
        self.declare_parameter("deadzone", 0.25)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("trajectory_dt", 0.25)
        self.declare_parameter("linear_speed", 0.10)  # m/s
        self.declare_parameter("fine_scale", 0.35)
        self.declare_parameter("axis_smoothing_alpha", 0.25)
        self.declare_parameter("max_joint_step", 0.25)
        self.declare_parameter("ik_seed_tolerance", 0.35)
        self.declare_parameter("invert.left_x", True)
        self.declare_parameter("invert.left_y", True)
        self.declare_parameter("invert.right_y", False)
        self.declare_parameter("gripper_open", 0.04)
        self.declare_parameter("gripper_close", 0.0)
        self.declare_parameter("gripper_effort", 40.0)
        self.declare_parameter("base_frame", "openarm_body_link0")
        self.declare_parameter("left_ee_link", "openarm_left_hand")
        self.declare_parameter("right_ee_link", "openarm_right_hand")

        self.declare_parameter("axis.left_x", 0)
        self.declare_parameter("axis.left_y", 1)
        self.declare_parameter("axis.right_y", 3)
        self.declare_parameter("button.x", 0)
        self.declare_parameter("button.y", 3)
        self.declare_parameter("button.lb", 4)
        self.declare_parameter("button.rb", 5)
        self.declare_parameter("button.lt", 6)
        self.declare_parameter("button.rt", 7)

        device_path = str(self.get_parameter("device_path").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.trajectory_dt = float(self.get_parameter("trajectory_dt").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.fine_scale = float(self.get_parameter("fine_scale").value)
        self.axis_smoothing_alpha = float(self.get_parameter("axis_smoothing_alpha").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.ik_seed_tolerance = float(self.get_parameter("ik_seed_tolerance").value)
        self.invert_left_x = bool(self.get_parameter("invert.left_x").value)
        self.invert_left_y = bool(self.get_parameter("invert.left_y").value)
        self.invert_right_y = bool(self.get_parameter("invert.right_y").value)
        self.gripper_open = float(self.get_parameter("gripper_open").value)
        self.gripper_close = float(self.get_parameter("gripper_close").value)
        self.gripper_effort = float(self.get_parameter("gripper_effort").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.left_ee_link = str(self.get_parameter("left_ee_link").value)
        self.right_ee_link = str(self.get_parameter("right_ee_link").value)

        self.axis_left_x = int(self.get_parameter("axis.left_x").value)
        self.axis_left_y = int(self.get_parameter("axis.left_y").value)
        self.axis_right_y = int(self.get_parameter("axis.right_y").value)
        self.btn_x = int(self.get_parameter("button.x").value)
        self.btn_y = int(self.get_parameter("button.y").value)
        self.btn_lb = int(self.get_parameter("button.lb").value)
        self.btn_rb = int(self.get_parameter("button.rb").value)
        self.btn_lt = int(self.get_parameter("button.lt").value)
        self.btn_rt = int(self.get_parameter("button.rt").value)

        self.left_joint_names = [f"openarm_left_joint{i}" for i in range(1, 8)]
        self.right_joint_names = [f"openarm_right_joint{i}" for i in range(1, 8)]
        self.left_group = "left_arm"
        self.right_group = "right_arm"
        self.left_gripper_name = "openarm_left_finger_joint1"
        self.right_gripper_name = "openarm_right_finger_joint1"

        self.left_pub = self.create_publisher(
            JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", 10
        )
        self.right_pub = self.create_publisher(
            JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", 10
        )
        self.left_gripper_client = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )
        self.right_gripper_client = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        self.current_joint_map: Dict[str, float] = {}
        self.arm_mode = "both"
        self._prev_x = 0
        self._prev_y = 0
        self._prev_lt = 0
        self._prev_rt = 0
        self._prev_rb = 0
        self._pending_ik_requests = 0
        self._ik_fail_count = {"left": 0, "right": 0}
        self._ik_last_warn_time = {"left": 0.0, "right": 0.0}
        self._last_jump_warn_time = {"left": 0.0, "right": 0.0}
        self._filt_axis = {"lx": 0.0, "ly": 0.0, "ry": 0.0}

        self.joy = LinuxJoystickReader(device_path=device_path)
        self.joy.start()
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            f"openarm_ee_teleop started: device={device_path} mode={self.arm_mode}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_map[name] = msg.position[i]

    def _axis(self, idx: int) -> float:
        if 0 <= idx < len(self.joy.axes):
            v = self.joy.axes[idx]
            return 0.0 if abs(v) < self.deadzone else v
        return 0.0

    def _button(self, idx: int) -> int:
        if 0 <= idx < len(self.joy.buttons):
            return self.joy.buttons[idx]
        return 0

    def _update_mode(self) -> None:
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

    def _send_gripper_goal(self, client: ActionClient, position: float) -> None:
        if not client.wait_for_server(timeout_sec=0.05):
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.gripper_effort)
        client.send_goal_async(goal)

    def _handle_gripper(self) -> None:
        lt = self._button(self.btn_lt)
        rt = self._button(self.btn_rt)

        # LT: close gripper, RT: open gripper
        if lt and not self._prev_lt:
            if self.arm_mode in ("left", "both"):
                self._send_gripper_goal(self.left_gripper_client, self.gripper_close)
            if self.arm_mode in ("right", "both"):
                self._send_gripper_goal(self.right_gripper_client, self.gripper_close)
        if rt and not self._prev_rt:
            if self.arm_mode in ("left", "both"):
                self._send_gripper_goal(self.left_gripper_client, self.gripper_open)
            if self.arm_mode in ("right", "both"):
                self._send_gripper_goal(self.right_gripper_client, self.gripper_open)

        self._prev_lt = lt
        self._prev_rt = rt

    def _go_home_if_needed(self) -> bool:
        rb = self._button(self.btn_rb)
        if rb and not self._prev_rb:
            home = [0.0] * 7
            if self.arm_mode in ("left", "both"):
                self._publish_traj(self.left_pub, self.left_joint_names, home)
            if self.arm_mode in ("right", "both"):
                self._publish_traj(self.right_pub, self.right_joint_names, home)
            self.get_logger().info("home command sent (all selected joints -> 0.0)")
            self._prev_rb = rb
            return True
        self._prev_rb = rb
        return False

    def _lookup_pose(self, ee_link: str) -> Optional[Pose]:
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, ee_link, rclpy.time.Time())
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        return pose

    def _build_ik_request(
        self, group_name: str, ee_link: str, pose: Pose, seed_names: List[str]
    ) -> GetPositionIK.Request:
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = group_name
        ik_req.ik_link_name = ee_link
        ik_req.pose_stamped = PoseStamped()
        ik_req.pose_stamped.header.frame_id = self.base_frame
        ik_req.pose_stamped.pose = pose
        ik_req.timeout.sec = 0
        ik_req.timeout.nanosec = 150_000_000

        state = RobotState()
        state.joint_state.name = list(seed_names)
        state.joint_state.position = [self.current_joint_map.get(n, 0.0) for n in seed_names]
        ik_req.robot_state = state

        # Keep seed joints near current state for smoother teleop.
        constraints = Constraints()
        for name in seed_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = self.current_joint_map.get(name, 0.0)
            jc.tolerance_above = self.ik_seed_tolerance
            jc.tolerance_below = self.ik_seed_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        ik_req.constraints = constraints

        req.ik_request = ik_req
        return req

    def _publish_traj(self, pub, joint_names: List[str], joint_values: List[float]) -> None:
        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_values
        sec = int(self.trajectory_dt)
        nsec = int((self.trajectory_dt - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec
        msg.points = [point]
        pub.publish(msg)

    def _extract_solution(self, arm: str, names: List[str], ik_res) -> Optional[List[float]]:
        if ik_res.error_code.val != 1:
            self._ik_fail_count[arm] += 1
            now = time.time()
            if now - self._ik_last_warn_time[arm] > 1.0:
                self.get_logger().warn(
                    f"{arm} IK failed: error_code={ik_res.error_code.val}, "
                    f"consecutive_failures={self._ik_fail_count[arm]}"
                )
                self._ik_last_warn_time[arm] = now
            return None
        solved = {}
        for i, n in enumerate(ik_res.solution.joint_state.name):
            if i < len(ik_res.solution.joint_state.position):
                solved[n] = ik_res.solution.joint_state.position[i]
        if any(n not in solved for n in names):
            self._ik_fail_count[arm] += 1
            now = time.time()
            if now - self._ik_last_warn_time[arm] > 1.0:
                self.get_logger().warn(
                    f"{arm} IK incomplete solution: "
                    f"consecutive_failures={self._ik_fail_count[arm]}"
                )
                self._ik_last_warn_time[arm] = now
            return None
        if self._ik_fail_count[arm] > 0:
            self.get_logger().info(
                f"{arm} IK recovered after {self._ik_fail_count[arm]} failures"
            )
            self._ik_fail_count[arm] = 0
        joints = [solved[n] for n in names]
        curr = [self.current_joint_map.get(n, 0.0) for n in names]
        max_delta = max(abs(a - b) for a, b in zip(joints, curr))
        if max_delta > self.max_joint_step:
            now = time.time()
            if now - self._last_jump_warn_time[arm] > 1.0:
                self.get_logger().warn(
                    f"{arm} IK jump rejected: max_joint_delta={max_delta:.3f} rad "
                    f"(limit={self.max_joint_step:.3f})"
                )
                self._last_jump_warn_time[arm] = now
            return None
        return joints

    def _smooth_axis(self, key: str, value: float) -> float:
        a = min(max(self.axis_smoothing_alpha, 0.0), 1.0)
        prev = self._filt_axis[key]
        out = prev + a * (value - prev)
        self._filt_axis[key] = out
        return out

    def _compute_target_poses(self) -> Tuple[Optional[Pose], Optional[Pose]]:
        axis_lx = self._axis(self.axis_left_x)
        axis_ly = self._axis(self.axis_left_y)
        axis_ry = self._axis(self.axis_right_y)

        if self.invert_left_x:
            axis_lx = -axis_lx
        if self.invert_left_y:
            axis_ly = -axis_ly
        if self.invert_right_y:
            axis_ry = -axis_ry

        axis_lx = self._smooth_axis("lx", axis_lx)
        axis_ly = self._smooth_axis("ly", axis_ly)
        axis_ry = self._smooth_axis("ry", axis_ry)

        speed_scale = self.fine_scale if self._button(self.btn_lb) else 1.0
        dx = axis_ly * self.linear_speed * speed_scale / self.publish_rate
        dy = axis_lx * self.linear_speed * speed_scale / self.publish_rate
        dz = -axis_ry * self.linear_speed * speed_scale / self.publish_rate
        if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
            return None, None

        left_pose = self._lookup_pose(self.left_ee_link)
        right_pose = self._lookup_pose(self.right_ee_link)
        if left_pose is not None:
            left_pose.position.x += dx
            left_pose.position.y += dy
            left_pose.position.z += dz
        if right_pose is not None:
            right_pose.position.x += dx
            right_pose.position.y += dy
            right_pose.position.z += dz
        return left_pose, right_pose

    def _on_timer(self) -> None:
        self._update_mode()
        self._handle_gripper()
        if self._go_home_if_needed():
            return
        if self._pending_ik_requests > 0:
            return
        if not self.ik_client.wait_for_service(timeout_sec=0.01):
            return
        left_pose, right_pose = self._compute_target_poses()
        if left_pose is None and right_pose is None:
            return

        if self.arm_mode in ("left", "both") and left_pose is not None:
            self._pending_ik_requests += 1
            req = self._build_ik_request(self.left_group, self.left_ee_link, left_pose, self.left_joint_names)
            fut = self.ik_client.call_async(req)
            fut.add_done_callback(self._left_ik_done)

        if self.arm_mode in ("right", "both") and right_pose is not None:
            self._pending_ik_requests += 1
            req = self._build_ik_request(
                self.right_group, self.right_ee_link, right_pose, self.right_joint_names
            )
            fut = self.ik_client.call_async(req)
            fut.add_done_callback(self._right_ik_done)

    def _left_ik_done(self, future) -> None:
        try:
            res = future.result()
            if res is None:
                return
            joints = self._extract_solution("left", self.left_joint_names, res)
            if joints is not None:
                self._publish_traj(self.left_pub, self.left_joint_names, joints)
        finally:
            self._pending_ik_requests = max(0, self._pending_ik_requests - 1)

    def _right_ik_done(self, future) -> None:
        try:
            res = future.result()
            if res is None:
                return
            joints = self._extract_solution("right", self.right_joint_names, res)
            if joints is not None:
                self._publish_traj(self.right_pub, self.right_joint_names, joints)
        finally:
            self._pending_ik_requests = max(0, self._pending_ik_requests - 1)

    def destroy_node(self) -> bool:
        self.joy.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenArmEETeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

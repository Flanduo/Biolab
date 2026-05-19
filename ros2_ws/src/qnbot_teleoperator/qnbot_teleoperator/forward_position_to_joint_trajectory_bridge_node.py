#!/usr/bin/env python3
"""
Bridge node: forward_position_controller commands -> joint_trajectory_controller action.

Goal:
  qnbot_teleoperator (and related teleop tooling) publish joint positions as:
    /left_forward_position_controller/commands  (Float64MultiArray, 7 positions)
    /right_forward_position_controller/commands (Float64MultiArray, 7 positions)

But joint_trajectory_controller expects trajectories via a FollowJointTrajectory action:
    /left_joint_trajectory_controller/follow_joint_trajectory
    /right_joint_trajectory_controller/follow_joint_trajectory

This node subscribes the Float64MultiArray command topics and continuously sends
single-point FollowJointTrajectory goals (streaming style) to the action servers.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ForwardPositionToJointTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__('forward_position_to_joint_trajectory_bridge_node')

        # ---- Topics to subscribe (produced by qnbot_teleoperator) ----
        self.declare_parameter('left_cmd_topic', '/left_forward_position_controller/commands')
        self.declare_parameter('right_cmd_topic', '/right_forward_position_controller/commands')

        # ---- Action servers to send (consumed by joint_trajectory_controller) ----
        self.declare_parameter(
            'left_action_name', '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.declare_parameter(
            'right_action_name', '/right_joint_trajectory_controller/follow_joint_trajectory'
        )

        # ---- Topic interface (fallback) ----
        # If the action server is not available, JointTrajectoryController typically
        # exposes a topic interface at:
        #   /<controller_name>/joint_trajectory
        self.declare_parameter(
            'left_joint_trajectory_topic',
            '/left_joint_trajectory_controller/joint_trajectory',
        )
        self.declare_parameter(
            'right_joint_trajectory_topic',
            '/right_joint_trajectory_controller/joint_trajectory',
        )

        # Controller joints order (must match joint_trajectory_controller config "joints:")
        self.declare_parameter(
            'left_joint_names',
            [
                'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
                'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
                'openarm_left_joint7',
            ],
        )
        self.declare_parameter(
            'right_joint_names',
            [
                'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
                'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
                'openarm_right_joint7',
            ],
        )

        # ---- Streaming parameters ----
        self.declare_parameter('send_rate_hz', 20.0)  # how often to send new goals
        self.declare_parameter('point_duration_sec', 0.05)  # time_from_start for each goal point
        self.declare_parameter(
            'command_timeout_sec',
            0.5,
        )  # stop sending if no new commands for this duration
        self.declare_parameter(
            'send_on_change_only',
            True,
        )
        self.declare_parameter(
            'position_change_threshold',
            1e-3,  # rad
        )

        # ---- Enable/disable arms independently ----
        self.declare_parameter('enable_left', True)
        self.declare_parameter('enable_right', True)
        self.declare_parameter('prefer_action', True)
        self.declare_parameter('publish_to_topic_on_action_unavailable', True)

        self.left_cmd_topic: str = self.get_parameter('left_cmd_topic').value
        self.right_cmd_topic: str = self.get_parameter('right_cmd_topic').value
        self.left_action_name: str = self.get_parameter('left_action_name').value
        self.right_action_name: str = self.get_parameter('right_action_name').value
        self.left_joint_trajectory_topic: str = self.get_parameter(
            'left_joint_trajectory_topic'
        ).value
        self.right_joint_trajectory_topic: str = self.get_parameter(
            'right_joint_trajectory_topic'
        ).value

        self.left_joint_names: List[str] = list(self.get_parameter('left_joint_names').value)
        self.right_joint_names: List[str] = list(self.get_parameter('right_joint_names').value)

        self.send_rate_hz: float = float(self.get_parameter('send_rate_hz').value)
        self.point_duration_sec: float = float(self.get_parameter('point_duration_sec').value)
        self.command_timeout_sec: float = float(self.get_parameter('command_timeout_sec').value)
        self.send_on_change_only: bool = bool(self.get_parameter('send_on_change_only').value)
        self.position_change_threshold: float = float(self.get_parameter('position_change_threshold').value)

        self.enable_left: bool = bool(self.get_parameter('enable_left').value)
        self.enable_right: bool = bool(self.get_parameter('enable_right').value)
        self.prefer_action: bool = bool(self.get_parameter('prefer_action').value)
        self.publish_to_topic_on_action_unavailable: bool = bool(
            self.get_parameter('publish_to_topic_on_action_unavailable').value
        )

        # Basic sanity checks
        if len(self.left_joint_names) != 7:
            self.get_logger().warn('Expected left_joint_names length = 7')
        if len(self.right_joint_names) != 7:
            self.get_logger().warn('Expected right_joint_names length = 7')

        # Latest positions from Float64MultiArray commands
        self.left_latest: Optional[List[float]] = None
        self.right_latest: Optional[List[float]] = None
        self.left_last_time = None
        self.right_last_time = None

        self._last_sent_left: Optional[List[float]] = None
        self._last_sent_right: Optional[List[float]] = None

        # Avoid spamming: at most one inflight goal per arm
        self._left_inflight = False
        self._right_inflight = False

        self.left_action_client = ActionClient(self, FollowJointTrajectory, self.left_action_name)
        self.right_action_client = ActionClient(self, FollowJointTrajectory, self.right_action_name)

        # Topic publishers (fallback)
        self.left_joint_traj_pub = self.create_publisher(
            JointTrajectory,
            self.left_joint_trajectory_topic,
            10,
        )
        self.right_joint_traj_pub = self.create_publisher(
            JointTrajectory,
            self.right_joint_trajectory_topic,
            10,
        )

        # Subscriptions (qnbot outputs this format)
        self.left_sub = self.create_subscription(
            Float64MultiArray,
            self.left_cmd_topic,
            self._left_cmd_cb,
            10,
        )
        self.right_sub = self.create_subscription(
            Float64MultiArray,
            self.right_cmd_topic,
            self._right_cmd_cb,
            10,
        )

        # Streaming timer
        interval = 1.0 / self.send_rate_hz if self.send_rate_hz > 0 else 0.05
        self.timer = self.create_timer(interval, self._timer_cb)

        self.get_logger().info('Forward -> JointTrajectory bridge started')
        self.get_logger().info(f'  left_cmd_topic   : {self.left_cmd_topic}')
        self.get_logger().info(f'  left_action_name : {self.left_action_name}')
        self.get_logger().info(f'  right_cmd_topic   : {self.right_cmd_topic}')
        self.get_logger().info(f'  right_action_name : {self.right_action_name}')
        self.get_logger().info(f'  send_rate_hz={self.send_rate_hz}, point_duration_sec={self.point_duration_sec}')
        self.get_logger().info(
            f'  prefer_action={self.prefer_action}, '
            f'publish_to_topic_on_action_unavailable={self.publish_to_topic_on_action_unavailable}'
        )

        # Debug throttling
        self._dbg_no_left_cmd_counter = 0
        self._dbg_no_right_cmd_counter = 0
        self._dbg_left_action_not_ready_counter = 0
        self._dbg_right_action_not_ready_counter = 0
        self._dbg_left_publish_counter = 0
        self._dbg_right_publish_counter = 0

    def _left_cmd_cb(self, msg: Float64MultiArray) -> None:
        if not self.enable_left:
            return
        if len(msg.data) < 7:
            self.get_logger().warn('Left cmd received with data length < 7; ignoring')
            return

        positions = [float(x) for x in msg.data[:7]]
        self.left_latest = positions
        self.left_last_time = self.get_clock().now()
        # Keep this log lightweight (prints at most every ~1s when sending stream)
        if self._dbg_no_left_cmd_counter > 0:
            self._dbg_no_left_cmd_counter = 0

    def _right_cmd_cb(self, msg: Float64MultiArray) -> None:
        if not self.enable_right:
            return
        if len(msg.data) < 7:
            self.get_logger().warn('Right cmd received with data length < 7; ignoring')
            return

        positions = [float(x) for x in msg.data[:7]]
        self.right_latest = positions
        self.right_last_time = self.get_clock().now()
        if self._dbg_no_right_cmd_counter > 0:
            self._dbg_no_right_cmd_counter = 0

    def _positions_changed(self, last: Optional[List[float]], current: List[float]) -> bool:
        if last is None:
            return True
        if len(last) != len(current):
            return True
        max_abs = max(abs(a - b) for a, b in zip(last, current))
        return max_abs > self.position_change_threshold

    def _send_single_point_goal(
        self,
        side: str,
        client: ActionClient,
        joint_names: List[str],
        positions: List[float],
    ) -> None:
        self.get_logger().info(
            f'[{side.upper()}] Sending FollowJointTrajectory goal: '
            f'positions={["{:.3f}".format(x) for x in positions]}, '
            f'time_from_start={self.point_duration_sec:.3f}s'
        )
        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance = Duration(sec=0, nanosec=0)

        traj = JointTrajectory()
        # Some controllers use trajectory header stamp for timing/synchronization.
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self.point_duration_sec),
            nanosec=int((self.point_duration_sec - int(self.point_duration_sec)) * 1e9),
        )
        traj.points = [point]

        goal.trajectory = traj

        future = client.send_goal_async(goal)
        if side == 'left':
            self._left_inflight = True
        else:
            self._right_inflight = True

        def _done_callback(fut) -> None:
            accepted = False
            try:
                goal_handle = fut.result()
                accepted = bool(getattr(goal_handle, 'accepted', False))
            except Exception:
                accepted = False

            if side == 'left':
                self._left_inflight = False
                if accepted:
                    self._last_sent_left = positions
            else:
                self._right_inflight = False
                if accepted:
                    self._last_sent_right = positions

            if not accepted:
                self.get_logger().warn(f'[{side}] FollowJointTrajectory goal not accepted')
            else:
                self.get_logger().info(f'[{side}] FollowJointTrajectory goal accepted')

        future.add_done_callback(_done_callback)

    def _publish_single_point_trajectory(
        self,
        side: str,
        joint_names: List[str],
        positions: List[float],
    ) -> None:
        """
        Publish to the topic interface of JointTrajectoryController:
          /<controller_name>/joint_trajectory
        """
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self.point_duration_sec),
            nanosec=int((self.point_duration_sec - int(self.point_duration_sec)) * 1e9),
        )
        traj.points = [point]

        if side == 'left':
            self.left_joint_traj_pub.publish(traj)
            self._last_sent_left = positions
            self._dbg_left_publish_counter += 1
        else:
            self.right_joint_traj_pub.publish(traj)
            self._last_sent_right = positions
            self._dbg_right_publish_counter += 1

        topic = self.left_joint_trajectory_topic if side == 'left' else self.right_joint_trajectory_topic
        if side == 'left':
            if self._dbg_left_publish_counter % 10 == 0:
                self.get_logger().info(
                    f'[LEFT] Action not used; publishing JointTrajectory to {topic}'
                )
        else:
            if self._dbg_right_publish_counter % 10 == 0:
                self.get_logger().info(
                    f'[RIGHT] Action not used; publishing JointTrajectory to {topic}'
                )

    def _timer_cb(self) -> None:
        now = self.get_clock().now()

        # ---- Left arm ----
        if self.enable_left:
            if self.left_latest is not None and self.left_last_time is not None:
                age = (now - self.left_last_time).nanoseconds / 1e9
                if age <= self.command_timeout_sec:
                    if not self._left_inflight:
                        if (not self.send_on_change_only) or self._positions_changed(
                            self._last_sent_left, self.left_latest
                        ):
                            if self.prefer_action and self.left_action_client.server_is_ready():
                                self._send_single_point_goal(
                                    side='left',
                                    client=self.left_action_client,
                                    joint_names=self.left_joint_names,
                                    positions=self.left_latest,
                                )
                            elif self.publish_to_topic_on_action_unavailable:
                                self._publish_single_point_trajectory(
                                    side='left',
                                    joint_names=self.left_joint_names,
                                    positions=self.left_latest,
                                )
                            # else: let next timer try
                # else: command too old, do nothing (expect next command to refresh last_time)
            else:
                self._dbg_no_left_cmd_counter += 1
                # print about every ~1-2s (depends on timer freq)
                if self._dbg_no_left_cmd_counter % int(max(1, (1.5 * self.send_rate_hz))) == 0:
                    self.get_logger().warn(
                        '[LEFT] No recent cmd yet (waiting for Float64MultiArray on '
                        f'{self.left_cmd_topic}).'
                    )

        # ---- Right arm ----
        if self.enable_right:
            if self.right_latest is not None and self.right_last_time is not None:
                age = (now - self.right_last_time).nanoseconds / 1e9
                if age <= self.command_timeout_sec:
                    if not self._right_inflight:
                        if (not self.send_on_change_only) or self._positions_changed(
                            self._last_sent_right, self.right_latest
                        ):
                            if self.prefer_action and self.right_action_client.server_is_ready():
                                self._send_single_point_goal(
                                    side='right',
                                    client=self.right_action_client,
                                    joint_names=self.right_joint_names,
                                    positions=self.right_latest,
                                )
                            elif self.publish_to_topic_on_action_unavailable:
                                self._publish_single_point_trajectory(
                                    side='right',
                                    joint_names=self.right_joint_names,
                                    positions=self.right_latest,
                                )
                # else: command too old, do nothing
            else:
                self._dbg_no_right_cmd_counter += 1
                if self._dbg_no_right_cmd_counter % int(max(1, (1.5 * self.send_rate_hz))) == 0:
                    self.get_logger().warn(
                        '[RIGHT] No recent cmd yet (waiting for Float64MultiArray on '
                        f'{self.right_cmd_topic}).'
                    )

        # Action readiness debug (throttled)
        if self.enable_left and self.left_action_client is not None and not self.left_action_client.server_is_ready():
            self._dbg_left_action_not_ready_counter += 1
            if self._dbg_left_action_not_ready_counter % int(max(1, (2.0 * self.send_rate_hz))) == 0:
                self.get_logger().warn(f'[LEFT] Action server not ready: {self.left_action_name}')
        if self.enable_right and self.right_action_client is not None and not self.right_action_client.server_is_ready():
            self._dbg_right_action_not_ready_counter += 1
            if self._dbg_right_action_not_ready_counter % int(max(1, (2.0 * self.send_rate_hz))) == 0:
                self.get_logger().warn(f'[RIGHT] Action server not ready: {self.right_action_name}')


def main() -> None:
    rclpy.init()
    node = ForwardPositionToJointTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


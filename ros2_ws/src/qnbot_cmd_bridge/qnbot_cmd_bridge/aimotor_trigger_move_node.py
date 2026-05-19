#!/usr/bin/env python3
"""
When /aimotor/position_state drops below threshold, publish arm target joints once.

This node is designed for simple threshold-trigger behavior:
- Subscribe: std_msgs/msg/Float64 (default: /aimotor/position_state)
- Publish:   std_msgs/msg/Float64MultiArray (default: /right_forward_position_controller/commands)
- Target joints can be loaded from a JointState-like text file (e.g. 1.md) by joint names.
"""

from __future__ import annotations

import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class AimotorTriggerMoveNode(Node):
    def __init__(self) -> None:
        super().__init__("aimotor_trigger_move_node")

        # I/O
        self.declare_parameter("state_topic", "/aimotor/position_state")
        self.declare_parameter(
            "right_command_topic", "/right_forward_position_controller/commands"
        )
        self.declare_parameter(
            "left_command_topic", "/left_forward_position_controller/commands"
        )
        self.declare_parameter("publish_left", True)
        self.declare_parameter("publish_right", True)

        # Trigger logic
        self.declare_parameter("trigger_threshold", 0.2)
        self.declare_parameter("release_threshold", 0.25)
        self.declare_parameter("cooldown_sec", 1.0)
        self.declare_parameter("enabled", True)

        # Target positions
        self.declare_parameter(
            "target_joint_names",
            [
                "openarm_right_joint1",
                "openarm_right_joint2",
                "openarm_right_joint3",
                "openarm_right_joint4",
                "openarm_right_joint5",
                "openarm_right_joint6",
                "openarm_right_joint7",
            ],
        )
        self.declare_parameter("target_positions", [])
        self.declare_parameter("target_pose_file", "1.md")
        self.declare_parameter("publish_repeat", 3)
        self.declare_parameter("publish_repeat_interval_sec", 0.05)

        self.state_topic: str = self.get_parameter("state_topic").value
        self.right_command_topic: str = self.get_parameter("right_command_topic").value
        self.left_command_topic: str = self.get_parameter("left_command_topic").value
        self.publish_left: bool = bool(self.get_parameter("publish_left").value)
        self.publish_right: bool = bool(self.get_parameter("publish_right").value)
        self.trigger_threshold: float = float(
            self.get_parameter("trigger_threshold").value
        )
        self.release_threshold: float = float(
            self.get_parameter("release_threshold").value
        )
        self.cooldown_sec: float = float(self.get_parameter("cooldown_sec").value)
        self.enabled: bool = bool(self.get_parameter("enabled").value)
        self.target_joint_names: List[str] = list(
            self.get_parameter("target_joint_names").value
        )
        self.target_positions: List[float] = [
            float(v) for v in self.get_parameter("target_positions").value
        ]
        self.target_pose_file: str = str(self.get_parameter("target_pose_file").value)
        self.publish_repeat: int = int(self.get_parameter("publish_repeat").value)
        self.publish_repeat_interval_sec: float = float(
            self.get_parameter("publish_repeat_interval_sec").value
        )

        if self.release_threshold < self.trigger_threshold:
            self.get_logger().warn(
                "release_threshold < trigger_threshold, auto set to trigger_threshold."
            )
            self.release_threshold = self.trigger_threshold

        if not self.target_positions:
            loaded = self._load_target_positions_from_file(
                self.target_pose_file, self.target_joint_names
            )
            if loaded is not None:
                self.target_positions = loaded

        if len(self.target_positions) != len(self.target_joint_names):
            self.get_logger().error(
                "target_positions length mismatch: "
                f"{len(self.target_positions)} vs {len(self.target_joint_names)}"
            )

        self.left_cmd_pub = self.create_publisher(
            Float64MultiArray, self.left_command_topic, 10
        )
        self.right_cmd_pub = self.create_publisher(
            Float64MultiArray, self.right_command_topic, 10
        )
        self.state_sub = self.create_subscription(
            Float64, self.state_topic, self._on_state_msg, 10
        )

        self._armed = True
        self._last_trigger_time: Optional[rclpy.time.Time] = None

        self.get_logger().info(
            "aimotor_trigger_move_node started: "
            f"state_topic={self.state_topic}, "
            f"left_command_topic={self.left_command_topic}, "
            f"right_command_topic={self.right_command_topic}"
        )
        self.get_logger().info(
            f"threshold(trigger/release)=({self.trigger_threshold}, {self.release_threshold}), "
            f"target_len={len(self.target_positions)}"
        )

    def _load_target_positions_from_file(
        self, pose_file: str, wanted_joint_names: List[str]
    ) -> Optional[List[float]]:
        resolved = pose_file
        if not os.path.isabs(resolved):
            resolved = os.path.join(os.getcwd(), resolved)

        if not os.path.isfile(resolved):
            self.get_logger().warn(
                f"target_pose_file not found: {resolved}. Use target_positions instead."
            )
            return None

        try:
            with open(resolved, "r", encoding="utf-8") as f:
                lines = [line.rstrip("\n") for line in f]
        except OSError as exc:
            self.get_logger().error(f"Failed to read target_pose_file: {exc}")
            return None

        names, positions = self._extract_jointstate_lists(lines)
        if not names or not positions or len(names) != len(positions):
            self.get_logger().error(
                f"Invalid JointState text in {resolved}: names/positions not matched."
            )
            return None

        table: Dict[str, float] = {n: float(p) for n, p in zip(names, positions)}
        missing = [n for n in wanted_joint_names if n not in table]
        if missing:
            self.get_logger().error(
                f"target_pose_file missing joints: {missing}. "
                "Please check target_joint_names or file content."
            )
            return None

        result = [table[n] for n in wanted_joint_names]
        self.get_logger().info(
            f"Loaded target positions from {resolved} by joint names successfully."
        )
        return result

    @staticmethod
    def _extract_jointstate_lists(lines: List[str]) -> Tuple[List[str], List[float]]:
        names: List[str] = []
        positions: List[float] = []
        section: Optional[str] = None

        for raw in lines:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("name:"):
                section = "name"
                continue
            if line.startswith("position:"):
                section = "position"
                continue
            if line.endswith(":") and line[:-1] in ("header", "velocity", "effort"):
                section = None
                continue

            if line.startswith("- "):
                item = line[2:].strip()
                if section == "name":
                    names.append(item)
                elif section == "position":
                    try:
                        positions.append(float(item))
                    except ValueError:
                        continue

        return names, positions

    def _on_state_msg(self, msg: Float64) -> None:
        if not self.enabled:
            return
        if len(self.target_positions) != len(self.target_joint_names):
            return

        value = float(msg.data)
        now = self.get_clock().now()

        if value > self.release_threshold:
            self._armed = True
            return

        if value >= self.trigger_threshold:
            return
        if not self._armed:
            return

        if self._last_trigger_time is not None:
            dt = (now - self._last_trigger_time).nanoseconds / 1e9
            if dt < self.cooldown_sec:
                return

        out = Float64MultiArray()
        out.data = self.target_positions
        repeat = max(1, self.publish_repeat)
        interval = max(0.0, self.publish_repeat_interval_sec)

        for i in range(repeat):
            if self.publish_left:
                self.left_cmd_pub.publish(out)
            if self.publish_right:
                self.right_cmd_pub.publish(out)
            if i < repeat - 1 and interval > 0.0:
                time.sleep(interval)

        self._last_trigger_time = now
        self._armed = False
        self.get_logger().warn(
            f"Triggered move: value={value:.4f} < {self.trigger_threshold:.4f}, "
            f"published {repeat} command(s), "
            f"left={self.publish_left}, right={self.publish_right}."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AimotorTriggerMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

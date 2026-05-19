#!/usr/bin/env python3
"""
Trigger both arms move when /aimotor/position_state is below threshold.

Usage example:
  python3 src/openarm_teleop/script/aimotor_trigger_move_bimanual.py \
    --config src/openarm_teleop/config/aimotor_trigger_move.yaml
"""

from __future__ import annotations

import argparse
import os
import time
from typing import Dict, List

import yaml

import rclpy
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class AimotorTriggerMoveBimanual(Node):
    def __init__(
        self,
        state_topic: str,
        left_topic: str,
        right_topic: str,
        trigger_threshold: float,
        release_threshold: float,
        restore_threshold: float,
        cooldown_sec: float,
        publish_repeat: int,
        publish_repeat_interval_sec: float,
        left_positions: List[float],
        right_positions: List[float],
        move_duration_sec: float,
        move_steps: int,
        restore_on_release: bool,
        use_right_target_for_both: bool,
    ) -> None:
        super().__init__("aimotor_trigger_move_bimanual")

        self.state_topic = state_topic
        self.left_topic = left_topic
        self.right_topic = right_topic
        # 触发阈值：低于该值执行目标动作
        self.trigger_threshold = float(trigger_threshold)
        # release_threshold 仅做兼容保留，不参与实际恢复判定
        self.release_threshold = float(release_threshold)
        # 恢复阈值：与“上升趋势”配合使用，见 _on_state 中恢复条件说明
        self.restore_threshold = float(restore_threshold)
        self.cooldown_sec = cooldown_sec
        self.publish_repeat = max(1, publish_repeat)
        self.publish_repeat_interval_sec = max(0.0, publish_repeat_interval_sec)

        self.left_positions = [float(v) for v in left_positions]
        self.right_positions = [float(v) for v in right_positions]
        self.restore_on_release = bool(restore_on_release)
        self.use_right_target_for_both = bool(use_right_target_for_both)
        if len(self.left_positions) != 7 or len(self.right_positions) != 7:
            raise ValueError("left_positions and right_positions must both be length 7")
        if self.use_right_target_for_both:
            self.left_positions = list(self.right_positions)
        self.move_duration_sec = max(0.0, float(move_duration_sec))
        self.move_steps = max(1, int(move_steps))

        self.left_joint_names = [
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
        ]
        self.right_joint_names = [
            "openarm_right_joint1",
            "openarm_right_joint2",
            "openarm_right_joint3",
            "openarm_right_joint4",
            "openarm_right_joint5",
            "openarm_right_joint6",
            "openarm_right_joint7",
        ]
        self._tracked_joint_names = self.left_joint_names + self.right_joint_names
        self._joint_pos_map: Dict[str, float] = {}
        self._last_joint_snapshot: Dict[str, float] = {}
        self._last_joint_move_time = self.get_clock().now()
        # 关节运动判定阈值：任一关节相邻两帧变化超过该值，视为“仍在运动”
        self._joint_delta_moving_threshold = 0.002
        # 从最近一次检测到运动起，至少静止这么久才允许触发，避免抢控制
        self._joint_settle_sec = 0.30

        self.left_pub = self.create_publisher(Float64MultiArray, self.left_topic, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, self.right_topic, 10)
        self.sub = self.create_subscription(Float64, self.state_topic, self._on_state, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        self._last_trigger_time = None
        self._active_low_state = False
        self._is_executing_motion = False
        self._saved_left_positions = list(self.left_positions)
        self._saved_right_positions = list(self.right_positions)
        self._state_msg_count = 0
        self._last_state_value = None
        self._prev_state_value = None
        self._last_debug_log_time = self.get_clock().now()
        self._no_msg_timer = self.create_timer(2.0, self._warn_if_no_state)

        self.get_logger().info(
            "Started. state_topic=%s, left_topic=%s, right_topic=%s"
            % (self.state_topic, self.left_topic, self.right_topic)
        )
        self.get_logger().info(
            "threshold(trigger/release/restore)=(%.4f, %.4f, %.4f)"
            % (
                self.trigger_threshold,
                self.release_threshold,
                self.restore_threshold,
            )
        )
        self.get_logger().info(
            "判定模式: 下行穿越 trigger 触发；回升时(上升且>restore)或(穿越 restore)或(上升且>trigger)则恢复"
        )
        self.get_logger().info(
            "move_duration_sec=%.3f, move_steps=%d"
            % (self.move_duration_sec, self.move_steps)
        )

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        n = min(len(msg.name), len(msg.position))
        changed = False
        for i in range(n):
            name = msg.name[i]
            pos = float(msg.position[i])
            self._joint_pos_map[name] = pos
            if name in self._tracked_joint_names:
                prev = self._last_joint_snapshot.get(name)
                if prev is not None and abs(pos - prev) > self._joint_delta_moving_threshold:
                    changed = True
                self._last_joint_snapshot[name] = pos
        if changed:
            self._last_joint_move_time = self.get_clock().now()

    def _is_robot_settled(self) -> bool:
        # 若关键关节还没读全，先认为未稳定，防止盲触发
        if len(self._last_joint_snapshot) < len(self._tracked_joint_names):
            return False
        dt = (self.get_clock().now() - self._last_joint_move_time).nanoseconds / 1e9
        return dt >= self._joint_settle_sec

    def _get_current_arm_positions(
        self, joint_names: List[str], fallback: List[float]
    ) -> List[float]:
        out: List[float] = []
        for i, name in enumerate(joint_names):
            if name in self._joint_pos_map:
                out.append(self._joint_pos_map[name])
            else:
                out.append(float(fallback[i]))
        return out

    def _publish_smooth(self, target_left: List[float], target_right: List[float]) -> None:
        self._is_executing_motion = True
        try:
            current_left = self._get_current_arm_positions(
                self.left_joint_names, target_left
            )
            current_right = self._get_current_arm_positions(
                self.right_joint_names, target_right
            )

            if self.move_duration_sec > 0.0 and self.move_steps > 1:
                dt = self.move_duration_sec / float(self.move_steps)
                for s in range(1, self.move_steps + 1):
                    a = float(s) / float(self.move_steps)
                    left_msg = Float64MultiArray()
                    right_msg = Float64MultiArray()
                    left_msg.data = [
                        current_left[i] + (target_left[i] - current_left[i]) * a
                        for i in range(7)
                    ]
                    right_msg.data = [
                        current_right[i] + (target_right[i] - current_right[i]) * a
                        for i in range(7)
                    ]
                    self.left_pub.publish(left_msg)
                    self.right_pub.publish(right_msg)
                    if s < self.move_steps and dt > 0.0:
                        time.sleep(dt)
            else:
                left_msg = Float64MultiArray()
                left_msg.data = target_left
                right_msg = Float64MultiArray()
                right_msg.data = target_right
                for i in range(self.publish_repeat):
                    self.left_pub.publish(left_msg)
                    self.right_pub.publish(right_msg)
                    if i < self.publish_repeat - 1 and self.publish_repeat_interval_sec > 0.0:
                        time.sleep(self.publish_repeat_interval_sec)
        finally:
            self._is_executing_motion = False

    def _on_state(self, msg: Float64) -> None:
        value = float(msg.data)
        now = self.get_clock().now()
        self._state_msg_count += 1
        prev = self._prev_state_value
        self._prev_state_value = value
        self._last_state_value = value

        # 避免在执行轨迹期间再次触发/恢复，导致与当前动作“抢控制”
        if self._is_executing_motion:
            return

        # 用关节状态判定“机械臂是否还在动”，未稳定则不做触发/恢复判定
        if not self._is_robot_settled():
            return

        if (now - self._last_debug_log_time).nanoseconds > int(1e9):
            self.get_logger().info(
                "state=%.4f, low_active=%s, trigger=%.4f, restore=%.4f"
                % (
                    value,
                    self._active_low_state,
                    self.trigger_threshold,
                    self.restore_threshold,
                )
            )
            self._last_debug_log_time = now

        # 仅在“下行穿越 trigger”时触发：prev >= trigger 且 value < trigger
        crossed_trigger_down = (
            prev is not None
            and prev >= self.trigger_threshold
            and value < self.trigger_threshold
        )
        if (not self._active_low_state) and crossed_trigger_down:
            if self._last_trigger_time is not None:
                dt = (now - self._last_trigger_time).nanoseconds / 1e9
                if dt < self.cooldown_sec:
                    return
            self._saved_left_positions = self._get_current_arm_positions(
                self.left_joint_names, self.left_positions
            )
            self._saved_right_positions = self._get_current_arm_positions(
                self.right_joint_names, self.right_positions
            )
            self._publish_smooth(self.left_positions, self.right_positions)
            self._active_low_state = True
            self._last_trigger_time = now
            self.get_logger().warn(
                "Triggered: state=%.4f < %.4f, moved to target pose."
                % (value, self.trigger_threshold)
            )
            return

        # 上升恢复（三种情况，避免漏恢复）：
        # 1) 上升趋势且高于 restore：value>prev 且 value>restore
        # 2) 从下向上穿越 restore：prev<=restore 且 value>restore
        # 3) 明显回升到触发线以上：value>prev 且 value>trigger（解决“卡在 14 直接跳到 17”时 prev 不满足 2 的情况）
        rising_and_over_restore = (
            prev is not None
            and value > prev
            and value > self.restore_threshold
        )
        crossed_restore_up = (
            prev is not None
            and prev <= self.restore_threshold
            and value > self.restore_threshold
        )
        rising_back_above_trigger = (
            prev is not None
            and value > prev
            and value > self.trigger_threshold
        )
        if self._active_low_state and (
            rising_and_over_restore
            or crossed_restore_up
            or rising_back_above_trigger
        ):
            if self.restore_on_release:
                self._publish_smooth(
                    self._saved_left_positions, self._saved_right_positions
                )
                self.get_logger().warn(
                    "Released: prev=%.4f -> curr=%.4f (restore=%.4f, trigger=%.4f), restored pre-trigger pose."
                    % (prev, value, self.restore_threshold, self.trigger_threshold)
                )
            self._active_low_state = False

    def _warn_if_no_state(self) -> None:
        if self._state_msg_count > 0:
            return
        self.get_logger().warn(
            "No messages on %s yet. Please check publisher/type/hardware."
            % self.state_topic
        )


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--config",
        default="",
        help="YAML 配置文件路径（不填则自动查找）",
    )
    return p.parse_args()


def load_config(config_path: str) -> Dict:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    auto_candidates = [
        os.path.normpath(os.path.join(script_dir, "..", "config", "aimotor_trigger_move.yaml")),
        os.path.normpath(os.path.join(os.getcwd(), "src", "openarm_teleop", "config", "aimotor_trigger_move.yaml")),
    ]

    resolved = ""
    if config_path:
        resolved = config_path
        if not os.path.isabs(resolved):
            resolved = os.path.join(os.getcwd(), resolved)
        resolved = os.path.normpath(resolved)
        if not os.path.isfile(resolved):
            raise FileNotFoundError(f"config file not found: {resolved}")
    else:
        for p in auto_candidates:
            if os.path.isfile(p):
                resolved = p
                break
        if not resolved:
            raise FileNotFoundError(
                "config file not found. Tried: " + ", ".join(auto_candidates)
            )

    with open(resolved, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError("invalid config format")
    return data


def main():
    args = parse_args()
    cfg = load_config(args.config)
    rclpy.init()
    node = AimotorTriggerMoveBimanual(
        state_topic=str(cfg.get("state_topic", "/aimotor/position_state")),
        left_topic=str(cfg.get("left_topic", "/left_forward_position_controller/commands")),
        right_topic=str(
            cfg.get("right_topic", "/right_forward_position_controller/commands")
        ),
        trigger_threshold=float(cfg.get("trigger_threshold", 0.2)),
        release_threshold=float(cfg.get("release_threshold", 0.25)),
        restore_threshold=float(cfg.get("restore_threshold", cfg.get("release_threshold", 0.25))),
        cooldown_sec=float(cfg.get("cooldown_sec", 1.0)),
        publish_repeat=int(cfg.get("publish_repeat", 3)),
        publish_repeat_interval_sec=float(cfg.get("publish_repeat_interval_sec", 0.05)),
        left_positions=list(cfg.get("left_positions", [])),
        right_positions=list(cfg.get("right_positions", [])),
        move_duration_sec=float(cfg.get("move_duration_sec", 2.0)),
        move_steps=int(cfg.get("move_steps", 50)),
        restore_on_release=bool(cfg.get("restore_on_release", True)),
        use_right_target_for_both=bool(cfg.get("use_right_target_for_both", True)),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy_ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

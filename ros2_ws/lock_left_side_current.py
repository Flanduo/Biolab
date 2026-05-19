#!/usr/bin/env python3
"""
Lock the left OpenArm arm and left LinkerHand at their current positions.

This intentionally uses the same control paths as the existing teleop/client code:
- left arm: /left_forward_position_controller/commands via rosbridge
- left hand: LinkerHand SDK, left O6 on can3
"""

import argparse
import signal
import sys
import time
from pathlib import Path
from threading import Event, Lock
from typing import Dict, List, Optional


PROJECT_ROOT = Path("/home/elwg/Biolab/openarm_demo")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from utils.linkerhand_control import LinkerHandControl
from utils.rosbridge_client import RosbridgeClient


NUM_ARM_JOINTS = 7
LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, NUM_ARM_JOINTS + 1)]
JOINT_STATES_TOPIC = "/joint_states"
JOINT_STATE_MSG_TYPE = "sensor_msgs/msg/JointState"
LEFT_ARM_TOPIC = "/left_forward_position_controller/commands"
CMD_MSG_TYPE = "std_msgs/msg/Float64MultiArray"
LEFT_ARM_RECORD_TOPIC = "/left_arm/joint_command"


class LeftArmReaderPublisher:
    def __init__(
        self,
        rosbridge_host: str,
        rosbridge_port: int,
        wait_timeout: float,
        require_hardware_feedback: bool,
        publish_record_command: bool,
    ):
        self.client = RosbridgeClient(host=rosbridge_host, port=rosbridge_port)
        self.client.on_connect = self._on_connect
        self.client.on_error = self._on_error
        self.wait_timeout = wait_timeout
        self.require_hardware_feedback = require_hardware_feedback
        self.publish_record_command = publish_record_command
        self._positions: Dict[str, float] = {}
        self._lock = Lock()
        self._have_left_arm = Event()

    def connect(self) -> None:
        if not self.client.connect():
            raise RuntimeError(f"failed to connect rosbridge at {self.client.url}")

    def _on_connect(self) -> None:
        self.client.subscribe(
            JOINT_STATES_TOPIC,
            JOINT_STATE_MSG_TYPE,
            self._on_joint_states,
            throttle_rate=20,
        )
        self.client.advertise(LEFT_ARM_TOPIC, CMD_MSG_TYPE)
        if self.publish_record_command:
            self.client.advertise(LEFT_ARM_RECORD_TOPIC, JOINT_STATE_MSG_TYPE)
        print(f"[ARM] rosbridge connected, publishing {LEFT_ARM_TOPIC}")
        if self.publish_record_command:
            print(f"[ARM] recording command publisher enabled: {LEFT_ARM_RECORD_TOPIC}")

    def _on_error(self, error: str) -> None:
        print(f"[ARM][ERROR] rosbridge: {error}")

    def _on_joint_states(self, msg: dict) -> None:
        if self.require_hardware_feedback:
            # Matches 10.0.0.2new.py: ignore display/merger messages with position only.
            if not msg.get("velocity") or not msg.get("effort"):
                return

        names = msg.get("name", [])
        positions = msg.get("position", [])
        if not names or not positions:
            return

        with self._lock:
            for name, pos in zip(names, positions):
                if name in LEFT_ARM_JOINTS:
                    self._positions[name] = float(pos)
            if all(name in self._positions for name in LEFT_ARM_JOINTS):
                self._have_left_arm.set()

    def capture_current_pose(self) -> List[float]:
        print(f"[ARM] waiting for left arm /joint_states, timeout={self.wait_timeout:.1f}s")
        if not self._have_left_arm.wait(self.wait_timeout):
            missing = self._missing_joint_names()
            raise RuntimeError(
                "failed to read all left arm joints from /joint_states; "
                f"missing={missing}. If you intentionally use display/merged states, "
                "rerun with --allow-position-only."
            )

        with self._lock:
            pose = [self._positions[name] for name in LEFT_ARM_JOINTS]
        print("[ARM] captured current pose: " + format_float_list(pose))
        return pose

    def publish_pose(self, pose: List[float]) -> None:
        msg = {"layout": {"dim": [], "data_offset": 0}, "data": [float(v) for v in pose]}
        self.client.publish(LEFT_ARM_TOPIC, CMD_MSG_TYPE, msg)
        if self.publish_record_command:
            stamp = ros_time_now_msg()
            record_msg = {
                "header": {"stamp": stamp, "frame_id": ""},
                "name": list(LEFT_ARM_JOINTS),
                "position": [float(v) for v in pose],
                "velocity": [0.0] * NUM_ARM_JOINTS,
                "effort": [0.0] * NUM_ARM_JOINTS,
            }
            self.client.publish(LEFT_ARM_RECORD_TOPIC, JOINT_STATE_MSG_TYPE, record_msg)

    def close(self) -> None:
        try:
            self.client.unadvertise(LEFT_ARM_TOPIC)
        except Exception:
            pass
        if self.publish_record_command:
            try:
                self.client.unadvertise(LEFT_ARM_RECORD_TOPIC)
            except Exception:
                pass
        try:
            self.client.disconnect()
        except Exception:
            pass

    def _missing_joint_names(self) -> List[str]:
        with self._lock:
            return [name for name in LEFT_ARM_JOINTS if name not in self._positions]


class LeftHandLocker:
    def __init__(
        self,
        hand_joint: str,
        can: str,
        wait_timeout: float,
        allow_zero: bool,
        speed: Optional[int],
        torque: Optional[int],
    ):
        self.hand = LinkerHandControl("left", hand_joint, can=can)
        self.wait_timeout = wait_timeout
        self.allow_zero = allow_zero
        self.speed = speed
        self.torque = torque

    def connect(self) -> None:
        if not self.hand.connect():
            raise RuntimeError(f"failed to connect left hand {self.hand.hand_joint} on {self.hand.can}")

        joint_count = self.hand.num_joints
        if self.speed is not None:
            if not self.hand.set_speed([self.speed] * joint_count):
                print("[HAND][WARN] failed to set speed; continuing")

        if self.torque is not None:
            try:
                self.hand._hand.set_torque([self.torque] * joint_count)
            except Exception as exc:
                print(f"[HAND][WARN] failed to set torque: {exc}")

    def capture_current_pose(self) -> List[int]:
        deadline = time.monotonic() + self.wait_timeout
        last_state = None
        while time.monotonic() < deadline:
            state = self.hand.get_state()
            last_state = state
            pose = self._normalize_state(state)
            if pose is not None:
                print("[HAND] captured current pose: " + format_int_list(pose))
                return pose
            time.sleep(0.05)

        raise RuntimeError(
            f"failed to read reliable left hand state before timeout; last_state={last_state}. "
            "If all-zero is a valid hand pose, rerun with --allow-hand-zero."
        )

    def publish_pose(self, pose: List[int]) -> None:
        if not self.hand.move(pose, force=True):
            print("[HAND][WARN] failed to publish hold pose")

    def close(self) -> None:
        if self.hand.connected:
            self.hand.disconnect()

    def _normalize_state(self, state) -> Optional[List[int]]:
        if state is None:
            return None
        values = list(state)
        if len(values) < self.hand.num_joints:
            return None
        pose = [max(0, min(255, int(round(float(v))))) for v in values[: self.hand.num_joints]]
        if not self.allow_zero and all(v == 0 for v in pose):
            return None
        return pose


class LeftSideLocker:
    def __init__(
        self,
        arm: Optional[LeftArmReaderPublisher],
        hand: Optional[LeftHandLocker],
        arm_hz: float,
        hand_hz: float,
    ):
        if arm_hz <= 0.0:
            raise ValueError("--arm-hz must be > 0")
        if hand_hz <= 0.0:
            raise ValueError("--hand-hz must be > 0")
        self.arm = arm
        self.hand = hand
        self.arm_period = 1.0 / arm_hz
        self.hand_period = 1.0 / hand_hz
        self.running = True

    def stop(self) -> None:
        self.running = False

    def run(self, arm_pose: Optional[List[float]], hand_pose: Optional[List[int]], publish_once: bool) -> None:
        next_arm_tick = time.monotonic()
        next_hand_tick = time.monotonic()
        status_tick = time.monotonic() + 2.0

        print_conflict_notice(self.arm is not None, self.hand is not None)
        print("[LOCK] holding left side; Ctrl+C to stop")

        while self.running:
            now = time.monotonic()
            did_publish = False

            if self.arm is not None and arm_pose is not None and now >= next_arm_tick - 1e-9:
                self.arm.publish_pose(arm_pose)
                next_arm_tick += self.arm_period
                did_publish = True

            if self.hand is not None and hand_pose is not None and now >= next_hand_tick - 1e-9:
                self.hand.publish_pose(hand_pose)
                next_hand_tick += self.hand_period
                did_publish = True

            if publish_once:
                print("[LOCK] published one hold command set")
                return

            if now >= status_tick:
                status = []
                if arm_pose is not None:
                    status.append("arm=" + format_float_list(arm_pose))
                if hand_pose is not None:
                    status.append("hand=" + format_int_list(hand_pose))
                print("[LOCK] holding " + " ".join(status))
                status_tick = now + 2.0

            if not did_publish:
                wakeups = [status_tick]
                if self.arm is not None:
                    wakeups.append(next_arm_tick)
                if self.hand is not None:
                    wakeups.append(next_hand_tick)
                time.sleep(max(0.001, min(wakeups) - time.monotonic()))

            if self.arm is not None and next_arm_tick < time.monotonic() - self.arm_period:
                next_arm_tick = time.monotonic()
            if self.hand is not None and next_hand_tick < time.monotonic() - self.hand_period:
                next_hand_tick = time.monotonic()

    def close(self) -> None:
        if self.arm is not None:
            self.arm.close()
        if self.hand is not None:
            self.hand.close()


def parse_float_pose(raw: Optional[str], expected_len: int, name: str) -> Optional[List[float]]:
    if raw is None:
        return None
    values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"{name} requires {expected_len} comma-separated values")
    return values


def parse_int_pose(raw: Optional[str], expected_len: int, name: str) -> Optional[List[int]]:
    if raw is None:
        return None
    values = [max(0, min(255, int(round(float(item.strip()))))) for item in raw.split(",") if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"{name} requires {expected_len} comma-separated values")
    return values


def format_float_list(values: List[float]) -> str:
    return "[" + ", ".join(f"{v:.4f}" for v in values) + "]"


def format_int_list(values: List[int]) -> str:
    return "[" + ", ".join(str(v) for v in values) + "]"


def ros_time_now_msg() -> Dict[str, int]:
    now_ns = time.time_ns()
    return {"sec": now_ns // 1_000_000_000, "nanosec": now_ns % 1_000_000_000}


def print_conflict_notice(lock_arm: bool, lock_hand: bool) -> None:
    print("[LOCK] resource notice:")
    if lock_arm:
        print(f"[LOCK]   left arm command publisher: {LEFT_ARM_TOPIC}")
        print(f"[LOCK]   left arm recorder command publisher: {LEFT_ARM_RECORD_TOPIC}")
        print("[LOCK]   do not run another active left-arm publisher unless you intentionally want command arbitration")
    if lock_hand:
        print("[LOCK]   left hand command channel: LinkerHand left O6 can3")
        print("[LOCK]   do not run another left-hand finger_move process on the same CAN channel")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Lock left arm and left LinkerHand at current positions.")
    parser.add_argument("--rosbridge-host", default="localhost")
    parser.add_argument("--rosbridge-port", type=int, default=9090)
    parser.add_argument("--wait-joint-states", type=float, default=5.0)
    parser.add_argument("--wait-hand-state", type=float, default=3.0)
    parser.add_argument("--arm-hz", type=float, default=50.0)
    parser.add_argument("--hand-hz", type=float, default=20.0)
    parser.add_argument("--allow-position-only", action="store_true")
    parser.add_argument("--allow-hand-zero", action="store_true")
    parser.add_argument("--disable-arm", action="store_true")
    parser.add_argument("--disable-hand", action="store_true")
    parser.add_argument(
        "--no-record-command",
        action="store_true",
        help=f"do not publish {LEFT_ARM_RECORD_TOPIC} for robot_data_recorder",
    )
    parser.add_argument("--left-hand-joint", default="O6")
    parser.add_argument("--left-hand-can", default="can3")
    parser.add_argument("--hand-speed", type=int, default=255)
    parser.add_argument("--hand-torque", type=int, default=100)
    parser.add_argument("--skip-hand-settings", action="store_true")
    parser.add_argument("--arm-pose", default=None, help="manual left arm pose: 7 comma-separated radians")
    parser.add_argument("--hand-pose", default=None, help="manual left hand pose: comma-separated 0..255 values")
    parser.add_argument("--publish-once", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.disable_arm and args.disable_hand:
        print("[ERROR] both --disable-arm and --disable-hand were set")
        return 1

    arm = None
    hand = None
    locker = None

    try:
        if not args.disable_arm:
            arm = LeftArmReaderPublisher(
                rosbridge_host=args.rosbridge_host,
                rosbridge_port=args.rosbridge_port,
                wait_timeout=args.wait_joint_states,
                require_hardware_feedback=not args.allow_position_only,
                publish_record_command=not args.no_record_command,
            )
            arm.connect()

        if not args.disable_hand:
            speed = None if args.skip_hand_settings else args.hand_speed
            torque = None if args.skip_hand_settings else args.hand_torque
            hand = LeftHandLocker(
                hand_joint=args.left_hand_joint,
                can=args.left_hand_can,
                wait_timeout=args.wait_hand_state,
                allow_zero=args.allow_hand_zero,
                speed=speed,
                torque=torque,
            )
            hand.connect()

        arm_pose = parse_float_pose(args.arm_pose, NUM_ARM_JOINTS, "--arm-pose")
        if arm is not None and arm_pose is None:
            arm_pose = arm.capture_current_pose()

        expected_hand_len = hand.hand.num_joints if hand is not None else 0
        hand_pose = parse_int_pose(args.hand_pose, expected_hand_len, "--hand-pose") if hand is not None else None
        if hand is not None and hand_pose is None:
            hand_pose = hand.capture_current_pose()

        locker = LeftSideLocker(arm=arm, hand=hand, arm_hz=args.arm_hz, hand_hz=args.hand_hz)

        def handle_signal(signum, frame):
            print("\n[LOCK] stopping")
            locker.stop()

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

        locker.run(arm_pose=arm_pose, hand_pose=hand_pose, publish_once=args.publish_once)
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1
    finally:
        if locker is not None:
            locker.close()
        else:
            if arm is not None:
                arm.close()
            if hand is not None:
                hand.close()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Lock the left OpenArm at its current joint position via rosbridge.

Control path matches /home/elwg/Biolab/video_record/10.0.0.2new.py:
- Read openarm_left_joint1..7 from /joint_states.
- Publish the captured 7 positions to /left_forward_position_controller/commands.
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

from utils.rosbridge_client import RosbridgeClient


NUM_ARM_JOINTS = 7
LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, NUM_ARM_JOINTS + 1)]
JOINT_STATES_TOPIC = "/joint_states"
JOINT_STATE_MSG_TYPE = "sensor_msgs/msg/JointState"
LEFT_ARM_TOPIC = "/left_forward_position_controller/commands"
CMD_MSG_TYPE = "std_msgs/msg/Float64MultiArray"


class LeftArmCurrentLocker:
    def __init__(
        self,
        rosbridge_host: str,
        rosbridge_port: int,
        hz: float,
        wait_timeout: float,
        require_hardware_feedback: bool,
    ):
        if hz <= 0.0:
            raise ValueError("hz must be > 0")

        self.client = RosbridgeClient(host=rosbridge_host, port=rosbridge_port)
        self.client.on_connect = self._on_connect
        self.client.on_error = self._on_error
        self.hz = hz
        self.wait_timeout = wait_timeout
        self.require_hardware_feedback = require_hardware_feedback
        self._positions: Dict[str, float] = {}
        self._lock = Lock()
        self._have_left_arm = Event()
        self._running = True

    def _on_connect(self) -> None:
        self.client.subscribe(
            JOINT_STATES_TOPIC,
            JOINT_STATE_MSG_TYPE,
            self._on_joint_states,
            throttle_rate=20,
        )
        self.client.advertise(LEFT_ARM_TOPIC, CMD_MSG_TYPE)
        print(f"[INFO] connected rosbridge and advertised {LEFT_ARM_TOPIC}")

    def _on_error(self, error: str) -> None:
        print(f"[ERROR] rosbridge: {error}")

    def _on_joint_states(self, msg: dict) -> None:
        if self.require_hardware_feedback:
            # Same filter as 10.0.0.2new.py: ignore GUI/merger messages that only carry position.
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

    def connect(self) -> None:
        if not self.client.connect():
            raise RuntimeError(f"failed to connect rosbridge at {self.client.url}")

    def capture_current_pose(self) -> List[float]:
        print(f"[INFO] waiting for left arm /joint_states, timeout={self.wait_timeout:.1f}s")
        if not self._have_left_arm.wait(self.wait_timeout):
            missing = self._missing_joint_names()
            raise RuntimeError(
                "failed to read all left arm joints from /joint_states; "
                f"missing={missing}. If you intentionally use simulated/merged "
                "joint states without velocity/effort, rerun with --allow-position-only."
            )

        with self._lock:
            pose = [self._positions[name] for name in LEFT_ARM_JOINTS]
        print("[INFO] captured left arm pose: " + self._format_pose(pose))
        return pose

    def lock(self, pose: List[float], publish_once: bool = False) -> None:
        if len(pose) != NUM_ARM_JOINTS:
            raise ValueError(f"left arm pose must have {NUM_ARM_JOINTS} values")

        period = 1.0 / self.hz
        next_tick = time.monotonic()
        count = 0

        print(f"[INFO] locking left arm at {self.hz:.1f} Hz; Ctrl+C to stop")
        while self._running:
            self.publish_pose(pose)
            count += 1

            if publish_once:
                print("[INFO] published one lock command")
                return

            if count % max(1, int(self.hz * 2.0)) == 0:
                print(f"[INFO] holding: {self._format_pose(pose)}")

            next_tick += period
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

    def publish_pose(self, pose: List[float]) -> None:
        msg = {"layout": {"dim": [], "data_offset": 0}, "data": [float(v) for v in pose]}
        self.client.publish(LEFT_ARM_TOPIC, CMD_MSG_TYPE, msg)

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        try:
            self.client.unadvertise(LEFT_ARM_TOPIC)
        except Exception:
            pass
        try:
            self.client.disconnect()
        except Exception:
            pass

    def _missing_joint_names(self) -> List[str]:
        with self._lock:
            return [name for name in LEFT_ARM_JOINTS if name not in self._positions]

    @staticmethod
    def _format_pose(pose: List[float]) -> str:
        return "[" + ", ".join(f"{v:.4f}" for v in pose) + "]"


def parse_pose(raw: Optional[str]) -> Optional[List[float]]:
    if raw is None:
        return None
    values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if len(values) != NUM_ARM_JOINTS:
        raise ValueError(f"--pose requires {NUM_ARM_JOINTS} comma-separated values")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Lock left OpenArm at current position.")
    parser.add_argument("--rosbridge-host", default="localhost")
    parser.add_argument("--rosbridge-port", type=int, default=9090)
    parser.add_argument("--hz", type=float, default=50.0, help="hold command publish rate")
    parser.add_argument("--wait-joint-states", type=float, default=5.0)
    parser.add_argument(
        "--allow-position-only",
        action="store_true",
        help="accept /joint_states messages that do not include velocity/effort",
    )
    parser.add_argument(
        "--pose",
        default=None,
        help="optional manual pose: seven comma-separated joint positions in radians",
    )
    parser.add_argument("--publish-once", action="store_true", help="publish one command and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    locker = LeftArmCurrentLocker(
        rosbridge_host=args.rosbridge_host,
        rosbridge_port=args.rosbridge_port,
        hz=args.hz,
        wait_timeout=args.wait_joint_states,
        require_hardware_feedback=not args.allow_position_only,
    )

    def handle_signal(signum, frame):
        print("\n[INFO] stopping left arm lock")
        locker.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        manual_pose = parse_pose(args.pose)
        locker.connect()
        pose = manual_pose if manual_pose is not None else locker.capture_current_pose()
        if manual_pose is not None:
            print("[INFO] using manual left arm pose: " + locker._format_pose(pose))
        locker.lock(pose, publish_once=args.publish_once)
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1
    finally:
        locker.close()


if __name__ == "__main__":
    sys.exit(main())

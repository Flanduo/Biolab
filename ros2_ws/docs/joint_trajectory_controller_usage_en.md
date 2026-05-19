# JointTrajectoryController Usage (English)

This document explains how to use ROS 2 `joint_trajectory_controller/JointTrajectoryController` (from `ros2_controllers`) and how to publish motion data to it in your OpenArm workspace.

It also documents the **“publish data”** topics used by the existing trajectory record/play scripts in `openarm_teleop`.

---

## 1. What controller interface do you have?

In your project, the MoveIt controller mapping uses:

- **`left_joint_trajectory_controller`** as `FollowJointTrajectory`
- **`right_joint_trajectory_controller`** as `FollowJointTrajectory`

The controller config also uses position command interfaces (i.e., joint-space positions).

---

## 2. Topics / Action servers you should expect

### 2.1 JointTrajectoryController (Action)

For `JointTrajectoryController`, you send trajectories via a `FollowJointTrajectory` **action**.

Typical action names in this repo are:

- `/left_joint_trajectory_controller/follow_joint_trajectory`
- `/right_joint_trajectory_controller/follow_joint_trajectory`

To verify what is running on your machine:

```bash
ros2 action list | rg "left_joint_trajectory_controller|right_joint_trajectory_controller|follow_joint_trajectory"
```

### 2.2 Joint state feedback topic

Trajectory record/play and many debug flows depend on:

- `/joint_states` (`sensor_msgs/msg/JointState`)

To quickly inspect it:

```bash
ros2 topic echo /joint_states
```

---

## 3. Publish joint trajectory data to JointTrajectoryController

### Option A (recommended): Python `rclpy` ActionClient (FollowJointTrajectory)

Send a `FollowJointTrajectory.Goal` containing:

- `goal.trajectory.joint_names` (must match the controller’s `joints` list)
- `goal.trajectory.points[]` with `positions[]` and `time_from_start`

Example for the **left** arm (replace joint names and values if needed):

```python
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def send_left_goal():
    rclpy.init()
    node = rclpy.create_node("send_left_traj")

    action_name = "/left_joint_trajectory_controller/follow_joint_trajectory"
    client = ActionClient(node, FollowJointTrajectory, action_name)
    client.wait_for_server()

    goal = FollowJointTrajectory.Goal()
    goal.goal_time_tolerance = Duration(sec=0, nanosec=0)

    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = [
        "openarm_left_joint1",
        "openarm_left_joint2",
        "openarm_left_joint3",
        "openarm_left_joint4",
        "openarm_left_joint5",
        "openarm_left_joint6",
        "openarm_left_joint7",
    ]

    p = JointTrajectoryPoint()
    p.positions = [0.0, 0.1, 0.2, 0.3, 0.0, -0.1, 0.05]  # TODO
    p.time_from_start = Duration(sec=2, nanosec=0)
    goal.trajectory.points = [p]

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    node.get_logger().info(f"Result: {result_future.result().result}")

    rclpy.shutdown()

if __name__ == "__main__":
    send_left_goal()
```

### Option B: CLI quick check (optional)

You can also try `ros2 action send_goal` after confirming your action name and type with:

```bash
ros2 action list
ros2 action info /left_joint_trajectory_controller/follow_joint_trajectory
```

Then send a goal with `ros2 action send_goal ...` (the exact YAML/JSON structure depends on the action message fields).

---

## 4. “How to publish data” in this workspace (openarm_teleop scripts)

Your existing trajectory **record/play** tools publish **commands** to a **forward position controller** (not the action-based JointTrajectoryController).

### 4.1 Where the play node publishes arm commands

The player publishes:

- `/left_forward_position_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- `/right_forward_position_controller/commands` (`std_msgs/msg/Float64MultiArray`)

Expected data layout:

- `data` array length is 7
- order corresponds to:
  - left: `openarm_left_joint1 .. openarm_left_joint7`
  - right: `openarm_right_joint1 .. openarm_right_joint7`

### 4.2 Where the play node publishes gripper commands

The player sends gripper commands via actions:

- `/left_gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`)
- `/right_gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`)

### 4.3 Start/stop services used by the scripts

- `/stop_recording` (`std_srvs/srv/SetBool`)
- `/start_playback`, `/stop_playback` (`std_srvs/srv/SetBool`)

---

## 5. Run the existing record/play tools

### 5.1 Record a trajectory

Start the robot first, then:

```bash
ros2 run openarm_teleop record_joint_trajectory.py --output my_trajectory.json
```

Stop recording with `Ctrl+C` (or use the provided stop service).

### 5.2 Play back a trajectory

```bash
ros2 run openarm_teleop play_joint_trajectory.py --input my_trajectory.json
```

Useful flags:

- `--loop`
- `--speed <factor>` (e.g., `2.0`)
- `--start-time <seconds>`
- `--max-frequency <Hz>` (default `100`)

---

## 6. Quick troubleshooting

### 6.1 No data / recorder can’t subscribe

Check that joint state broadcaster is running:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

### 6.2 Robot doesn’t move during playback

Check that the forward position controllers are active and that command topics exist:

```bash
ros2 control list_controllers
ros2 topic list | rg "left_forward_position_controller/commands|right_forward_position_controller/commands"
```

---

## 7. Notes specific to JointTrajectoryController

If you choose to command `JointTrajectoryController` directly (Section 3):

- The `FollowJointTrajectory` goal must use the correct `joint_names` and include positions for all joints (your configs typically set `allow_partial_joints_goal: false`).
- Action-based control is generally independent from the forward-position command topics used by the record/play scripts.


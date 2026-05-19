# JointTrajectoryController 使用说明（中文）

本文说明在 OpenArm 工作区中如何使用 ROS 2 的 `joint_trajectory_controller/JointTrajectoryController`（来自 `ros2_controllers`），以及如何向它发布运动数据。

同时也会说明当前 `openarm_teleop` 轨迹录制/回放脚本所使用的“发布数据”话题。

---

## 1. 你当前使用的控制器接口

在你的工程里，MoveIt 控制器映射使用：

- **`left_joint_trajectory_controller`**（类型：`FollowJointTrajectory`）
- **`right_joint_trajectory_controller`**（类型：`FollowJointTrajectory`）

控制器配置使用的是位置命令接口（即关节空间位置）。

---

## 2. 你应当看到的 Topic / Action

### 2.1 JointTrajectoryController（Action）

对 `JointTrajectoryController` 来说，轨迹通常通过 `FollowJointTrajectory` **action** 发送。

本仓库常见 action 名称：

- `/left_joint_trajectory_controller/follow_joint_trajectory`
- `/right_joint_trajectory_controller/follow_joint_trajectory`

可用下面命令确认机器上实际运行的 action：

```bash
ros2 action list | rg "left_joint_trajectory_controller|right_joint_trajectory_controller|follow_joint_trajectory"
```

### 2.2 关节状态反馈 Topic

轨迹录制/回放和很多调试流程依赖：

- `/joint_states`（`sensor_msgs/msg/JointState`）

快速查看：

```bash
ros2 topic echo /joint_states
```

---

## 3. 向 JointTrajectoryController 发布轨迹数据

### 方案 A（推荐）：Python `rclpy` ActionClient（FollowJointTrajectory）

发送 `FollowJointTrajectory.Goal` 时，至少包含：

- `goal.trajectory.joint_names`（必须与控制器 `joints` 顺序一致）
- `goal.trajectory.points[]`（每个点包含 `positions[]` 和 `time_from_start`）

下面是**左臂**示例（按需替换关节名和目标值）：

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

### 方案 B：CLI 快速检查（可选）

你也可以先确认 action 名称与类型，再尝试 `ros2 action send_goal`：

```bash
ros2 action list
ros2 action info /left_joint_trajectory_controller/follow_joint_trajectory
```

然后再用 `ros2 action send_goal ...` 发送目标（具体 YAML/JSON 结构取决于 action 字段）。

---

## 4. 本工作区中的“如何发布数据”（openarm_teleop 脚本）

当前的轨迹**录制/回放**工具，默认是把命令发布到 **forward position controller**，不是直接发到 action 型的 JointTrajectoryController。

### 4.1 回放节点发布机械臂关节命令的位置

回放节点发布到：

- `/left_forward_position_controller/commands`（`std_msgs/msg/Float64MultiArray`）
- `/right_forward_position_controller/commands`（`std_msgs/msg/Float64MultiArray`）

数据约定：

- `data` 数组长度为 7
- 顺序对应：
  - 左臂：`openarm_left_joint1 .. openarm_left_joint7`
  - 右臂：`openarm_right_joint1 .. openarm_right_joint7`

### 4.2 回放节点发布夹爪命令的位置

夹爪通过 action 发送：

- `/left_gripper_controller/gripper_cmd`（`control_msgs/action/GripperCommand`）
- `/right_gripper_controller/gripper_cmd`（`control_msgs/action/GripperCommand`）

### 4.3 脚本使用的开始/停止服务

- `/stop_recording`（`std_srvs/srv/SetBool`）
- `/start_playback`、`/stop_playback`（`std_srvs/srv/SetBool`）

---

## 5. 运行现有录制/回放工具

### 5.1 录制轨迹

先启动机器人，再运行：

```bash
ros2 run openarm_teleop record_joint_trajectory.py --output my_trajectory.json
```

按 `Ctrl+C` 停止并保存（或使用停止服务）。

### 5.2 回放轨迹

```bash
ros2 run openarm_teleop play_joint_trajectory.py --input my_trajectory.json
```

常用参数：

- `--loop`
- `--speed <factor>`（例如 `2.0`）
- `--start-time <seconds>`
- `--max-frequency <Hz>`（默认 `100`）

---

## 6. 快速排查

### 6.1 没有数据 / 录制节点无法订阅

检查 joint state broadcaster 是否在运行：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

### 6.2 回放时机器人不动

检查 forward position controllers 是否 active，命令话题是否存在：

```bash
ros2 control list_controllers
ros2 topic list | rg "left_forward_position_controller/commands|right_forward_position_controller/commands"
```

---

## 7. JointTrajectoryController 相关补充说明

如果你要直接控制 `JointTrajectoryController`（见第 3 节）：

- `FollowJointTrajectory` 目标必须使用正确的 `joint_names`，并且包含所有关节位置（你的配置通常是 `allow_partial_joints_goal: false`）。
- action 控制链路与录制/回放脚本使用的 forward-position 话题链路通常是相互独立的。


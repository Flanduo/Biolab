# qnbot_cmd_bridge

将 **qnbot_teleoperator** 的 `/exo` 数据桥接到与 **f710_teleop** 相同的发布接口，使底盘和升降机构可以用同一套话题接收外骨骼/遥操作数据。

## 功能概览

- **订阅**（数据来源）：
  - `/exo/gamepad_keys`（`sensor_msgs/Joy`）：外骨骼摇杆与按钮
- **发布**（与 f710_teleop 一致）：
  - `/svtrobot_cmd`（`geometry_msgs/Twist`）：底盘速度  
    - 左摇杆前后 → `linear.x`，左摇杆左右 → `linear.y`，右摇杆左右 → `angular.z`
  - `/lift_control_cmd`（`std_msgs/Int32MultiArray`）：升降 `[direction, speed]`  
    - 左手柄 A=上升，B=下降；右手柄 B=急停
- **可选**：将 `/exo/joint_command` 转发到指定话题（默认关闭）。

只有在 qnbot 的 **小车/滑台控制** 启用（`/exo/gamepad_keys` 的 `buttons[3]==1`）时才会发布底盘与升降命令。

## 依赖

- ROS2（Humble 等）
- `rclpy`、`geometry_msgs`、`std_msgs`、`sensor_msgs`
- 需先运行 **qnbot_teleoperator** 的 `websocket_teleoperator`，保证 `/exo/gamepad_keys` 有数据。

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select qnbot_cmd_bridge
source install/setup.bash
```

### 启动桥接节点

```bash
ros2 launch qnbot_cmd_bridge exo_cmd_bridge.launch.py
```

参数来自 `config/exo_cmd_bridge.yaml`，可修改速度、死区、升降速度、按钮映射等。

### 直接运行节点

```bash
ros2 run qnbot_cmd_bridge exo_cmd_bridge_node --ros-args --params-file src/qnbot_cmd_bridge/config/exo_cmd_bridge.yaml
```

## 配置说明（config/exo_cmd_bridge.yaml）

| 参数 | 说明 | 默认 |
|------|------|------|
| `exo_gamepad_topic` | 外骨骼摇杆话题 | `/exo/gamepad_keys` |
| `cmd_vel_topic` / `lift_cmd_topic` | 底盘、升降发布话题 | `/svtrobot_cmd`, `/lift_control_cmd` |
| `velocity.max_linear` / `velocity.max_angular` | 线速度、角速度上限 | 1.0, 1.0 |
| `velocity.angular_scale` | 角速度缩放 | 1.0 |
| `invert.left_x` / `invert.right_x` | 左/右摇杆左右取反 | false |
| `disable_left_x_when_right_active` | 右摇杆有输入时禁用左摇杆左右 | false |
| `deadzone` | 摇杆死区 | 0.1 |
| `lift.speed` | 升降速度值 | 500 |
| `button.lift_up` / `button.lift_down` / `button.emergency` | 升/降/急停在 Joy.buttons 中的索引 | 5, 6, 12 |
| `cmd_vel.publish_always` | 是否始终按频率发布底盘（否则仅在有输入或松杆时发） | false |
| `forward_joint_command` | 是否转发 `/exo/joint_command` | false |

## 典型用法

1. 启动 qnbot WebSocket 遥操作：  
   `ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py`
2. 启动本桥接：  
   `ros2 launch qnbot_cmd_bridge exo_cmd_bridge.launch.py`
3. 底盘/升降节点订阅 `/svtrobot_cmd`、`/lift_control_cmd` 即可，与使用 f710_teleop 时相同。

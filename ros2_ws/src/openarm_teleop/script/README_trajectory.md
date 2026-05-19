# OpenArm关节轨迹录制和播放

## 功能说明

本工具提供了完整的关节轨迹录制和播放功能：

1. **录制功能** (`record_joint_trajectory.py`): 订阅关节状态话题，录制关节数据并保存到JSON文件
2. **播放功能** (`play_joint_trajectory.py`): 读取JSON文件，发布命令到控制器话题，重现录制的轨迹

## 使用方法

### 1. 录制轨迹

#### 方法1: 使用Python脚本
```bash
# 在启动机器人后，运行录制节点
ros2 run openarm_teleop record_joint_trajectory.py --output my_trajectory.json

# 或者直接运行Python脚本
cd ~/ros2_ws
python3 src/openarm_teleop/script/record_joint_trajectory.py --output my_trajectory.json
```

#### 方法2: 使用Shell脚本（Linux/Mac）
```bash
# 设置执行权限（首次使用）
chmod +x src/openarm_teleop/script/record_trajectory.sh

# 运行录制脚本
./src/openarm_teleop/script/record_trajectory.sh my_trajectory.json
```

#### 录制选项
- `--output, -o`: 指定输出文件路径（默认：`trajectory_YYYYMMDD_HHMMSS.json`）
- `--record-commands`: 同时录制命令话题（可选）

#### 控制录制
- **开始录制**: 节点启动后自动开始录制
- **停止录制**: 
  - 按 `Ctrl+C` 停止并保存
  - 或使用服务: `ros2 service call /stop_recording std_srvs/srv/SetBool '{data: true}'`

### 2. 播放轨迹

#### 方法1: 使用Python脚本
```bash
# 播放轨迹
ros2 run openarm_teleop play_joint_trajectory.py --input my_trajectory.json

# 或直接运行
python3 src/openarm_teleop/script/play_joint_trajectory.py --input my_trajectory.json
```

#### 方法2: 使用Shell脚本（Linux/Mac）
```bash
# 设置执行权限（首次使用）
chmod +x src/openarm_teleop/script/play_trajectory.sh

# 播放轨迹
./src/openarm_teleop/script/play_trajectory.sh my_trajectory.json
```

#### 播放选项
- `--input, -i`: 指定输入文件路径（必需）
- `--loop, -l`: 循环播放
- `--speed <factor>`: 播放速度因子（1.0=正常速度，2.0=2倍速，0.5=0.5倍速）
- `--start-time <seconds>`: 从指定时间（秒）开始播放
- `--max-frequency <Hz>`: 最大发布频率（默认100Hz，匹配控制器频率）

#### 控制播放
- **开始播放**: 节点启动后自动开始播放
- **停止播放**: 
  - 按 `Ctrl+C` 停止
  - 或使用服务: `ros2 service call /stop_playback std_srvs/srv/SetBool '{data: true}'`

## 完整使用示例

### 示例1: 录制并播放

```bash
# 1. 启动机器人
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller

# 2. 在另一个终端录制轨迹
cd ~/ros2_ws
python3 src/openarm_teleop/script/record_joint_trajectory.py --output demo.json
# 操作机器人，然后按Ctrl+C停止录制

# 3. 播放录制的轨迹
python3 src/openarm_teleop/script/play_joint_trajectory.py --input demo.json
```

### 示例2: 循环播放

```bash
python3 src/openarm_teleop/script/play_joint_trajectory.py --input demo.json --loop
```

### 示例3: 快速播放（2倍速）

```bash
python3 src/openarm_teleop/script/play_joint_trajectory.py --input demo.json --speed 2.0
```

### 示例4: 从中间开始播放

```bash
python3 src/openarm_teleop/script/play_joint_trajectory.py --input demo.json --start-time 5.0
```

## 数据格式

录制的JSON文件格式：

```json
{
  "metadata": {
    "created_at": "2025-01-XX...",
    "total_points": 1000,
    "duration": 20.5,
    "record_commands": false
  },
  "trajectory": [
    {
      "time": 0.0,
      "left_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "right_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "left_gripper": 0.0,
      "right_gripper": 0.0
    },
    ...
  ]
}
```

## 精确还原保证

播放节点采用以下机制确保精确还原原始动作：

1. **精确时间控制**: 使用 `time.perf_counter()` 高精度计时，基于原始时间戳计算播放时间
2. **时间误差补偿**: 自动检测并补偿时间误差，确保播放节奏准确
3. **频率匹配**: 默认最大发布频率100Hz，匹配控制器更新频率（10ms周期）
4. **高精度等待**: 对于短时间间隔使用忙等待，确保时间精度

### 播放模式说明

- **默认模式（speed=1.0）**: 完全按照录制的时间戳播放，精确还原原始动作
- **加速/减速模式**: 通过 `--speed` 参数调整播放速度，但保持动作的相对时间关系
- **频率限制**: 通过 `--max-frequency` 限制发布频率，避免超过控制器处理能力

## 注意事项

1. **录制前确保机器人已启动**: 录制节点需要订阅 `/joint_states` 话题
2. **播放前确保机器人已启动**: 播放节点需要发布到控制器话题
3. **关节数量**: 每个手臂7个关节 + 1个夹爪关节
4. **时间精度**: 录制的时间戳精度为秒（浮点数），播放时使用高精度计时还原
5. **文件大小**: 长时间录制可能产生较大的JSON文件
6. **动作还原**: 播放时会精确还原录制时的动作和时间节奏，确保动作一致性

## 故障排除

### 问题1: 找不到话题
```
错误: 无法订阅 /joint_states
```
**解决**: 确保机器人已启动，并且 `joint_state_broadcaster` 正在运行

### 问题2: 播放时机器人不动
```
错误: 无法发布命令
```
**解决**: 
- 检查控制器是否已启动
- 检查话题名称是否正确: `/left_forward_position_controller/commands` 和 `/right_forward_position_controller/commands`

### 问题3: 夹爪不工作
```
警告: 夹爪Action服务器未就绪
```
**解决**: 确保 `gripper_controller` 已启动

## 服务接口

### 录制节点服务
- `/start_recording` (std_srvs/srv/SetBool): 开始录制
- `/stop_recording` (std_srvs/srv/SetBool): 停止录制并保存

### 播放节点服务
- `/start_playback` (std_srvs/srv/SetBool): 开始播放
- `/stop_playback` (std_srvs/srv/SetBool): 停止播放

## 开发说明

- 录制节点订阅: `/joint_states` (sensor_msgs/JointState)
- 播放节点发布: 
  - `/left_forward_position_controller/commands` (std_msgs/Float64MultiArray)
  - `/right_forward_position_controller/commands` (std_msgs/Float64MultiArray)
  - 夹爪使用Action: `/left_gripper_controller/gripper_cmd` 和 `/right_gripper_controller/gripper_cmd`


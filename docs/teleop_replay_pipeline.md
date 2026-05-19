# 方案一：遥操采集 → 数据处理 → 真机复现

> 全流程文档，服务器 10.0.0.19 (Ubuntu 22.04, elwg)

## 流程概览

```
┌─────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌─────────────┐
│ 1. 启动硬件   │ →  │ 2. 遥操录制   │ →  │ 3. 数据对齐   │ →  │ 4. 数据清洗   │ →  │ 5. 真机复现  │
│ start_       │    │ robot_data_  │    │ vla_         │    │ fix_hand_    │    │ replay_to_  │
│ hardware.sh  │    │ recorder.py  │    │ preprocessor │    │ joints.py    │    │ robot.py    │
└─────────────┘    └──────────────┘    └──────────────┘    └──────────────┘    └─────────────┘
```

---

## 1. 启动硬件

### 1.1 一键启动

```bash
bash ~/Biolab/scripts/start_hardware.sh
```

该脚本会自动完成：
- 配置 4 路 CAN 设备 (can0~can3，需要 sudo 密码)
- 启动 ROS2 双臂驱动 (`openarm.bimanual.launch.py`)
- 启动 rosbridge (:9090)
- 启动 Viser 控制台 (:8082)

启动后通过 `http://10.0.0.19:8082` 访问控制界面。

### 1.2 停止硬件

```bash
tmux kill-session -t biolab_hardware
```

### 1.3 手动启动遥操作（6 步）

```bash
source /opt/ros/humble/setup.bash
source ~/Biolab/ros2_ws/install/setup.bash

# 步骤1: WebSocket 服务（接收外骨骼硬件数据）
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py

# 步骤2: 外骨骼显示
ros2 launch qnbot_teleoperator exoskeleton_display.launch.py source:=exo_command

# 步骤3: OpenArm 显示
ros2 launch qnbot_teleoperator openarm_display.launch.py use_exoskeleton:=true

# 步骤4: 外骨骼重定向
ros2 launch qnbot_teleoperator exo_retargeting.launch.py robot_type:=OpenArm

# 步骤5: 硬件控制器
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller

# 步骤6: 桥接控制
ros2 launch qnbot_teleoperator exoskeleton_bridge.launch.py gripper_scaling_factor:=0.05
```

### 1.4 硬件配置参考

| 设备 | 型号 | 接口 | CAN |
|------|------|------|-----|
| 左臂 | OpenArm v10 (7 DOF) | can0 | PEAK PCAN-USB FD |
| 右臂 | OpenArm v10 (7 DOF) | can1 | PEAK PCAN-USB FD |
| 左灵巧手 | LinkerHand O6 (6 DOF) | can3 | PEAK PCAN-USB |
| 右灵巧手 | LinkerHand O6 (6 DOF) | can2 | PEAK PCAN-USB |

**总自由度：36 DOF** (14 臂 + 22 手)

---

## 2. 遥操录制

### 2.1 启动录制

```bash
cd ~/Biolab/video_record
source ~/miniconda3/etc/profile.d/conda.sh
conda activate /home/elwg/Biolab/conda_envs/ros_env

# 独立录制
python3 robot_data_recorder.py

# 指定 session ID
python3 robot_data_recorder.py --session 20260506_222144

# 禁用灵巧手录制
python3 robot_data_recorder.py --no-hand

# 配合多路相机同步录制
python3 robot_data_recorder.py --sync --sync-port 5007
```

### 2.2 录制参数

| 参数 | 说明 |
|------|------|
| `--session <id>` | 自定义 session ID（默认自动生成时间戳） |
| `--sync` | 等待 UDP START 信号后开始录制 |
| `--sync-port <port>` | UDP 同步端口（默认 5007） |
| `--no-hand` | 禁用灵巧手数据录制 |

### 2.3 录制内容

录制节点以 **50Hz** 同步采样，同时采集以下数据：

| 数据 | 来源 | 话题/接口 |
|------|------|----------|
| 机械臂关节状态 | `joint_state_broadcaster` (真实编码器) | `/joint_states` |
| 机械臂关节指令 | `exo_retargeting_node` (遥操作映射) | `/left_arm/joint_command`, `/right_arm/joint_command` |
| 灵巧手关节状态 | LinkerHand SDK (CAN 直读) | can2, can3 |
| 灵巧手关节指令 | 上一拍手部状态差分 | CAN |

### 2.4 输出文件

录制完成后在 `video_record/robot_data/` 下生成 4 个 CSV 文件（扁平存放，按 session 时间戳区分）：

```
robot_data/
├── joint_states_<session>.csv          # 机械臂关节状态 (14 关节, 编码器真实值)
├── joint_commands_<session>.csv        # 机械臂关节指令 (14 关节, 遥操作指令)
├── hand_states_<session>.csv           # 灵巧手关节状态 (12 关节, CAN 直读)
└── hand_commands_<session>.csv         # 灵巧手关节指令 (12 关节)
```

> 所有文件行数严格 1:1，共享时间戳，50Hz 采样。

---

## 3. 数据对齐

### 3.1 仅 CSV 模式（不含相机图像）

```bash
cd ~/Biolab/video_record

# 自动查找 session 数据并处理
python3 vla_preprocessor.py --csv-only --auto-session 20260506_222144

# 指定输出目录
python3 vla_preprocessor.py --csv-only --auto-session 20260506_222144 --out /path/to/output
```

### 3.2 参数说明

| 参数 | 说明 |
|------|------|
| `--csv-only` | **不提取视频帧**，仅合并对齐 CSV 数据 |
| `--auto-session <id>` | 自动查找 session 对应的原始数据文件 |
| `--out <dir>` | 输出目录（默认 `processed_dataset/`） |
| `--skip <N>` | 每 N 帧取 1 帧（降采样） |
| `--size <px>` | 输出图像尺寸（默认 518，仅视频模式有效） |

### 3.3 处理流程

```
joint_states.csv  ─┐
joint_commands.csv ─┤
hand_states.csv   ─┼─→  时间戳对齐  →  去毛刺滤波  →  aligned_data.csv
hand_commands.csv ─┘
```

### 3.4 输出文件

```
processed_dataset/20260506_222144/
├── aligned_data.csv          # 对齐后的统一数据
├── arm_joint_curves.png      # 机械臂关节曲线图
└── hand_joint_curves.png     # 灵巧手关节曲线图
```

### 3.5 aligned_data.csv 列定义（共 52 列）

| 序号 | 列名 | 说明 |
|------|------|------|
| 0 | `frame_idx` | 帧索引 |
| 1 | `timestamp_ms` | 时间戳 (ms) |
| 2-8 | `openarm_left_joint1~7` | 左臂关节状态 (rad) |
| 9-15 | `openarm_right_joint1~7` | 右臂关节状态 (rad) |
| 16-22 | `cmd_openarm_left_joint1~7_action` | 左臂关节指令 (rad) |
| 23-29 | `cmd_openarm_right_joint1~7_action` | 右臂关节指令 (rad) |
| 30-35 | `left_hand_joint1~6` | 左手关节状态 |
| 36-41 | `right_hand_joint1~6` | 右手关节状态 |
| 42-47 | `left_hand_joint1~6_cmd_action` | 左手关节指令 |
| 48-53 | `right_hand_joint1~6_cmd_action` | 右手关节指令 |

> 灵巧手指令范围 0~255，0=完全弯曲，254=完全张开。

---

## 4. 数据清洗

### 4.1 使用修复脚本

```bash
cd ~/Biolab/video_record

# 将指定帧段的右手大拇指指令置0，所有帧的右手3~6指指令置0
python3 fix_hand_joints.py <csv_path> <range1_start> <range1_end> <range2_start> <range2_end>
```

### 4.2 示例

```bash
# 将 frame 380~450 和 1000~1080 的 right_hand_joint1_cmd_action 置 0
# 所有帧的 right_hand_joint3~6_cmd_action 置 0
python3 fix_hand_joints.py \
  processed_dataset/20260506_222144/aligned_data.csv \
  380 450 1000 1080
```

### 4.3 修改内容

| 操作 | 列 | 帧范围 | 值 |
|------|-----|--------|-----|
| 倒数第六列 → 0 | `right_hand_joint1_cmd_action` | range1 + range2 | 0 |
| 后四列 → 0 | `right_hand_joint3~6_cmd_action` | 全部帧 | 0 |

> 脚本直接原地修改 CSV，建议先备份。

---

## 5. 真机复现

### 5.1 基本复现

```bash
cd ~/Biolab/openarm_demo

# 全双臂 + 双手复现
python3 replay_to_robot.py ../video_record/processed_dataset/20260506_222144/aligned_data.csv

# 仅左臂
python3 replay_to_robot.py <csv_path> --arm left

# 0.5 倍速（慢放）
python3 replay_to_robot.py <csv_path> --speed 0.5

# 不控制灵巧手
python3 replay_to_robot.py <csv_path> --no-hand
```

### 5.2 增强复现（推荐）

```bash
cd ~/Biolab/openarm_demo

# 使用改进版复现脚本，支持触发模式和状态插值
python3 replay_csv_improved.py <csv_path>

# 触发模式（拇指弯曲/释放）
python3 replay_csv_improved.py <csv_path> --trigger

# 使用关节状态列（而非指令列）进行插值
python3 replay_csv_improved.py <csv_path> --use-state

# 自定义控制频率
python3 replay_csv_improved.py <csv_path> --control-hz 50 --hand-hz 5
```

### 5.3 独立灵巧手复现

可与 `--no-hand` 的臂复现并行运行：

```bash
cd ~/Biolab/openarm_demo

# 双手
python3 replay_hand_csv.py <csv_path>

# 仅左手
python3 replay_hand_csv.py <csv_path> --hand left
```

### 5.4 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--arm` | 控制哪只臂 (`both`/`left`/`right`) | `both` |
| `--speed` | 回放速度倍率 | `1.0` |
| `--no-hand` | 禁用灵巧手控制 | 否 |
| `--step-duration` | 无时间戳时的默认帧间隔 (s) | `2.0` |
| `--left-can` | 左手 CAN 通道 | `can3` |
| `--right-can` | 右手 CAN 通道 | `can2` |

### 5.5 安全注意事项

- 夡操作前确保机器人周围无障碍物
- 首次复现建议使用 `--speed 0.3` 慢速验证
- 复现前机械臂应在初始零位附近
- 灵巧手会在启动时发送张开指令 (`[254]*6`)

---

## 6. 3D 可视化预览（可选）

在真机复现前，可先在浏览器中预览动作：

```bash
cd ~/Biolab/openarm_demo

# 仅可视化
python3 replay_dataset.py <csv_path>

# 可视化 + 实时控制真机
python3 replay_dataset.py <csv_path> --real

# 可视化 + 真机 + 灵巧手
python3 replay_dataset.py <csv_path> --real --real-hand
```

访问 `http://10.0.0.19:8083` 查看 3D 可视化。

---

## 快速操作清单

```bash
# === 1. 启动硬件 ===
bash ~/Biolab/scripts/start_hardware.sh

# === 2. 遥操录制 ===
cd ~/Biolab/video_record
source ~/miniconda3/etc/profile.d/conda.sh
conda activate /home/elwg/Biolab/conda_envs/ros_env
python3 robot_data_recorder.py
# Ctrl+C 停止录制，记下 session ID

# === 3. 数据对齐 ===
cd ~/Biolab/video_record
python3 vla_preprocessor.py --csv-only --auto-session <session_id>

# === 4. 数据清洗 ===
cd ~/Biolab/video_record
python3 fix_hand_joints.py processed_dataset/<session_id>/aligned_data.csv <r1_start> <r1_end> <r2_start> <r2_end>

# === 5. 真机复现 ===
cd ~/Biolab/openarm_demo
python3 replay_to_robot.py ../video_record/processed_dataset/<session_id>/aligned_data.csv --speed 0.5
```

---

## 目录结构

```
~/Biolab/
├── video_record/
│   ├── robot_data_recorder.py       # 步骤2: 录制节点
│   ├── vla_preprocessor.py          # 步骤3: 数据对齐
│   ├── fix_hand_joints.py           # 步骤4: 数据清洗
│   ├── multi_cam_launcher.py        # 可选: 多路相机同步
│   ├── robot_data/                  # 录制原始数据 (扁平存放)
│   │   ├── joint_states_<session>.csv
│   │   ├── joint_commands_<session>.csv
│   │   ├── hand_states_<session>.csv
│   │   └── hand_commands_<session>.csv
│   └── processed_dataset/           # 处理后数据
│       └── <session_id>/
│           ├── aligned_data.csv
│           ├── arm_joint_curves.png
│           └── hand_joint_curves.png
├── openarm_demo/
│   ├── replay_to_robot.py           # 步骤5: 真机复现
│   ├── replay_csv_improved.py       # 步骤5: 增强版复现
│   ├── replay_dataset.py            # 可选: 3D 可视化
│   └── replay_hand_csv.py           # 可选: 独立灵巧手复现
└── scripts/
    └── start_hardware.sh            # 步骤1: 硬件启动
```

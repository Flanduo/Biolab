# Biolab 下位机数据采集、预处理与推理流程

本文档梳理 `/home/elwg/Biolab/video_record` 下位机侧脚本与 DexGraspVLA 项目的关系，用于后续设计“分步骤采集、分步骤训练、分步骤执行”的液体转移任务流程。

当前已验证访问路径：

```bash
ssh -J openarm@10.0.0.2 elwg@192.168.3.19
cd /home/elwg/Biolab/video_record
```

不要把密码写进项目文档。这里记录的是拓扑和路径：当前本机不能直接访问 `192.168.3.19`，需要通过 `10.0.0.2` 中转。

## 脚本职责总览

| 脚本 | 角色 | 输入 | 输出/动作 |
| --- | --- | --- | --- |
| `multi_cam_launcher.py` | 多相机录制启动器 | ZED、RealSense 设备 | 生成 `SESSION_ID`，启动 ZED/RealSense 录制，并向相机进程发送 UDP `START` |
| `robot_data_recorder.py` | 机器人状态与指令采集 | ROS2 `/joint_states`、左右臂 command topic、LinkerHand CAN | `robot_data/joint_states_*.csv`、`joint_commands_*.csv`、`hand_states_*.csv`、`hand_commands_*.csv` |
| `vla_preprocessor.py` | 离线数据对齐与 episode 生成 | 相机视频、相机时间戳、机器人 CSV | `processed_dataset/<SESSION_ID>/` 下的多相机图片和 `aligned_data.csv` |
| `10.0.0.2new.py` | 在线推理客户端 | 当前机器人状态、实时相机图像 | 上传观测到推理服务，接收 `action_seq`，可选择直接执行 |

这四个脚本对应两条链路：

```text
离线训练数据链路:
multi_cam_launcher.py + robot_data_recorder.py
    -> raw video / raw csv
    -> vla_preprocessor.py
    -> processed_dataset/<SESSION_ID>
    -> DexGraspVLA 标注 / zarr 构建 / 训练

在线部署链路:
10.0.0.2new.py
    -> 实时采集图像和机器人状态
    -> 请求远程推理服务
    -> 执行 26 维 action_seq
```

## 远端目录约定

当前脚本里的真实路径以 `/home/elwg/Biolab/video_record` 为准：

```text
/home/elwg/Biolab/video_record/
├── multi_cam_launcher.py
├── robot_data_recorder.py
├── vla_preprocessor.py
├── 10.0.0.2new.py
├── .current_session_id
├── ZEDProject/
│   └── video/
├── realsense/
│   └── recordings/
├── robot_data/
└── processed_dataset/
```

`video_record/README.md` 里的部分示例路径仍写成 `/home/openarm/...`，实际执行时应该使用 `/home/elwg/Biolab/...`。

## 离线采集流程

### 1. 多相机录制

启动脚本：

```bash
cd /home/elwg/Biolab/video_record
python3 multi_cam_launcher.py
```

脚本启动后会生成：

```text
SESSION_ID = YYYYMMDD_HHMMSS
/home/elwg/Biolab/video_record/.current_session_id
```

它会启动两个相机录制子进程：

| 相机 | 子脚本 | 同步端口 | ready 端口 | 输出 |
| --- | --- | --- | --- | --- |
| ZED | `ZEDProject/zed_recorder.py` | `5005` | `5010` | `ZEDProject/video/` 下的 SVO/SVO2 |
| RealSense | `realsense/record_realsense.py` | `5006` | `5010` | `realsense/recordings/cam0_<SESSION_ID>.mp4`、`cam1_<SESSION_ID>.mp4` 和 timestamps |

`multi_cam_launcher.py` 当前只等待 ZED 和 RealSense 的 `READY`，然后只向端口 `5005`、`5006` 发送 `START`。

### 2. 机器人数据录制

机器人数据采集依赖 ROS2 和 LinkerHand SDK：

```bash
source /opt/ros/humble/setup.bash
source /home/elwg/Biolab/ros2_ws/install/setup.bash
cd /home/elwg/Biolab/video_record
python3 robot_data_recorder.py --session <SESSION_ID>
```

如果在同一台机器上另开终端，可以用 launcher 写出的 session：

```bash
SESSION_ID=$(cat /home/elwg/Biolab/video_record/.current_session_id)
python3 robot_data_recorder.py --session "$SESSION_ID"
```

输出目录：

```text
/home/elwg/Biolab/video_record/robot_data/
├── joint_states_<SESSION_ID>.csv
├── joint_commands_<SESSION_ID>.csv
├── hand_states_<SESSION_ID>.csv
└── hand_commands_<SESSION_ID>.csv
```

机器人采集内容：

| 文件 | 频率/来源 | 内容 |
| --- | --- | --- |
| `joint_states_*.csv` | 约 50Hz，ROS2 `/joint_states` | 双臂真实关节状态，过滤掉 finger/gripper，要求消息含 velocity 和 effort |
| `joint_commands_*.csv` | 约 50Hz，ROS2 command topic | 左右臂 command，写成 `cmd_<joint_name>` |
| `hand_states_*.csv` | 约 50Hz，LinkerHand CAN | 左右手各 6 维状态，过滤无效和全 0 读取 |
| `hand_commands_*.csv` | 约 50Hz，由 hand state 后移得到 | `command[i] = state[i+1]`，时间戳使用上一帧 |

LinkerHand 当前配置：

```text
left  hand: can3, O6
right hand: can2, O6
```

### 3. 当前同步关系

当前相机链路是 UDP 同步启动的，但机器人 recorder 没有被 `multi_cam_launcher.py` 纳入同步启动：

```text
launcher waits: ZED + RealSense
launcher sends START: 5005 + 5006
robot_data_recorder --sync waits: 5007
```

也就是说，如果直接运行现有脚本，机器人数据通常依靠：

1. 使用相同 `SESSION_ID` 命名；
2. `vla_preprocessor.py` 后处理时按时间戳求交集裁剪；
3. 采集人员在相机开始后尽快启动/保持 robot recorder。

这能工作，但不是严格同步。更稳的后续改法是让 launcher 支持 robot recorder：

```text
expected_ready 加入 ROBOT
START 端口列表加入 5007
robot_data_recorder 使用 --sync --sync-port 5007 --session <SESSION_ID>
```

这样三个数据源都会在同一个 `START` 信号后开始。

## 原始数据格式

### 相机原始数据

RealSense：

```text
realsense/recordings/
├── cam0_<SESSION_ID>.mp4
├── cam0_<SESSION_ID>_timestamps.txt
├── cam1_<SESSION_ID>.mp4
└── cam1_<SESSION_ID>_timestamps.txt
```

ZED：

```text
ZEDProject/video/
└── 与 <SESSION_ID> 时间接近的 cam1/cam2 SVO 或 SVO2 文件
```

`vla_preprocessor.py` 对 ZED 文件使用模糊匹配，不完全依赖文件名完全等于 `SESSION_ID`。

### 机器人 CSV

臂状态：

```text
ros_timestamp_ns,system_timestamp_ns,openarm_left_joint1,...,openarm_right_joint7
```

臂指令：

```text
ros_timestamp_ns,system_timestamp_ns,cmd_openarm_left_joint1,...,cmd_openarm_right_joint7
```

手状态：

```text
system_timestamp_ns,left_hand_joint1,...,left_hand_joint6,right_hand_joint1,...,right_hand_joint6
```

手指令：

```text
system_timestamp_ns,left_hand_joint1_cmd,...,left_hand_joint6_cmd,right_hand_joint1_cmd,...,right_hand_joint6_cmd
```

## 预处理流程

推荐使用 auto-session：

```bash
cd /home/elwg/Biolab/video_record
python3 vla_preprocessor.py --auto-session <SESSION_ID>
```

可选抽帧：

```bash
python3 vla_preprocessor.py --auto-session <SESSION_ID> --skip 5
```

脚本会自动寻找：

```text
RealSense: realsense/recordings/cam0_<SESSION_ID>.mp4
RealSense: realsense/recordings/cam1_<SESSION_ID>.mp4
ZED:       ZEDProject/video/ 中与 session 时间接近的 cam1/cam2 文件
Robot:     robot_data/joint_states_<SESSION_ID>.csv 等四类 CSV
```

对齐逻辑：

1. 选择 master 时间轴，优先级为 `rs0 -> rs1 -> zed1 -> zed2`。
2. 计算所有相机和机器人 CSV 的时间交集。
3. 按 `--skip` 抽样 master 时间戳。
4. 每个采样时间戳上，对所有相机和 CSV 查找最近帧/最近行。
5. 写出图片和 `aligned_data.csv`。

输出结构：

```text
processed_dataset/<SESSION_ID>/
├── rs0/
│   ├── 000000.jpg
│   └── ...
├── rs1/
├── zed1/
├── zed2/
├── aligned_data.csv
├── arm_joint_curves.png
└── hand_joint_curves.png
```

`aligned_data.csv` 的列结构：

```text
frame_idx,timestamp_ms,
<arm_state_cols>,
<hand_state_cols>,
<arm_command_cols>_action,
<hand_command_cols>_action
```

注意：脚本里有 `--size` 参数和 `center_crop_resize()` 辅助函数，但当前主处理路径里图片是直接 `cv2.imwrite()` 写出的。训练侧如果需要固定尺寸，应在 DexGraspVLA 的 dataset transform 或后续预处理里统一处理。

还有一个纯 CSV 模式：

```bash
python3 vla_preprocessor.py \
  --csv-only \
  --auto-session <SESSION_ID>
```

这个模式只合并机器人 CSV，不抽取视频帧，适合检查机器人数据本身是否对齐。

## 数据采集任务管理表

采集任务、批次、episode 状态和后续处理进度单独维护在：

```text
docs/biolab_data_collection_table.md
```

当前第一批数据为 B001 / S1 抓取瓶子，2026-05-22 采集，已预处理并校验 28 个 episode、2216 帧，包含 `rs0`、`rs1`、`zed1` 三路图片；2026-05-24 已完成第一轮 DexGraspVLA 训练，best checkpoint 记录在采集任务台账中。

## 接入 DexGraspVLA 训练

`vla_preprocessor.py` 的输出是 DexGraspVLA 训练前的 episode 级原始数据。它应该按任务 step 放入对应的数据目录，例如：

```text
liquid_transfer_raw/
├── s1_bottle_grasp/
│   ├── 20260521_101010/
│   │   ├── rs0/
│   │   ├── rs1/
│   │   ├── zed1/
│   │   └── aligned_data.csv
├── s2_bottle_to_aspiration_pose/
├── s3_pipette_align_bottle/
├── s4_aspirate_from_bottle/
├── s5_transfer_to_tube/
└── s6_dispense_to_tube/
```

建议关系：

| 液体转移 step | 采集输出 | 训练方式 |
| --- | --- | --- |
| S1 抓取瓶子 | 独立 episodes | 单独训练瓶子抓取策略 |
| S2 瓶子移动到固定吸液位 | 独立 episodes | 单独训练瓶子搬运策略，把后续步骤转成固定位置单物体任务 |
| S3 右手移液器对准瓶口 | 独立 episodes | 单独训练 pipette 对准瓶口 |
| S4 吸取液体 | 独立 episodes 或程序化动作 | 可训练，也可后续用固定时长拇指关节动作替代 |
| S5 移液器移动到目标试管 | 独立 episodes | 单独训练 pipette 到试管架目标位 |
| S6 放液体 | 独立 episodes 或程序化动作 | 可训练，也可后续用固定时长拇指关节动作替代 |

DexGraspVLA 训练侧需要保持三个约束：

1. camera folder 的语义固定，例如 `zed1` 作为头部/主视角，`rs0`、`rs1` 作为左右腕视角。
2. `aligned_data.csv` 中 state/action 维度和训练配置一致。
3. 每个 step 的 checkpoint 配套自己的 normalization stats，在线推理时不能混用。

## 在线推理流程

`10.0.0.2new.py` 是部署侧客户端，不是离线采集脚本。它做的事情是：

```text
实时读取机器人状态和相机图像
    -> POST 到远程推理服务 /infer
    -> 接收 action_seq
    -> 可选执行动作
    -> 可选把 action_seq 写到 CSV
```

默认远程推理服务：

```text
http://10.0.0.18:9112
```

常用启动方式：

```bash
cd /home/elwg/Biolab/video_record
python3 10.0.0.2new.py --execute-action --loop --action-step-duration 0.3
```

如果只想看推理返回，不执行机器人：

```bash
python3 10.0.0.2new.py --action-dry-run
```

上传内容：

| 字段 | 内容 |
| --- | --- |
| `robot_state.arm.left/right` | 双臂 7+7 维当前状态 |
| `robot_state.hand.left/right` | 双手 6+6 维当前状态 |
| ZED image | 当前脚本字段名为 `zed_cam1_right` |
| RealSense images | `rs_cam0`、`rs_cam1` 等 |

推理返回动作：

```text
action_seq: T x 26
26 维 = left_arm(7) + left_hand(6) + right_arm(7) + right_hand(6)
```

当前执行逻辑：

1. 默认跳过 `action_seq` 前 2 帧；
2. 执行 `action_seq[2:8]`，也就是最多 6 个 action step；
3. 双臂通过 rosbridge 发布到 forward position controller；
4. 双手通过 LinkerHand CAN 下发 0-255 的关节命令；
5. 如果当前位置与第一帧动作差异大，会先 warmup 插值到第一帧。

默认 action log：

```text
/home/elwg/Biolab/video_record/processed_dataset/<timestamp>/aligned_data.csv
```

这个 log 主要记录在线推理返回的 action，不包含完整训练 episode 的图片和状态对齐结果，不能直接当作离线训练数据使用。

## 和多步骤任务衔接的关系

后续六步液体转移任务可以这样接：

```text
Step policy checkpoint
    -> 10.0.0.2new.py 执行动作
    -> 顶层多模态模型/规则判断 step 是否完成
    -> 切换到下一个 step checkpoint
```

瓶子移动到固定吸液位的设计是合理的：S2 结束后，瓶子位置固定，S3/S4 就不再需要同时处理“移动瓶子”和“对准瓶口”，任务复杂度下降。

吸液和放液如果最后决定走程序化动作，则训练数据里仍建议保留少量 S4/S6 episode，用于：

1. 验证视觉状态和实际液体操作时序；
2. 给顶层完成判定提供正负样本；
3. 后续如果程序动作不稳定，可以回退到训练策略。

多任务衔接建议明确三个接口：

| 接口 | 说明 |
| --- | --- |
| `start_condition` | 当前 step 开始时物体/手/机器人必须满足的状态 |
| `success_condition` | 顶层 Qwen 多模态模型或规则判断 step 完成 |
| `handover_state` | 传给下一 step 的固定状态，例如瓶子到吸液位、移液器到瓶口上方 |

每个 step 采集时都要覆盖自己的 `start_condition`，否则训练出的策略会依赖上一步的偶然状态，后续串联会不稳。

## 当前需要确认或修正的点

1. 机器人 recorder 尚未被 launcher 严格同步。正式大规模采集前，建议把 `5007` robot sync 加到 `multi_cam_launcher.py`。
2. `10.0.0.2new.py` 的 ZED 字段名是 `zed_cam1_right`，但脚本当前 capture 使用的是 left image，需要确认推理服务训练时到底使用哪一路。
3. RealSense 的 `cam0/cam1` 依赖设备枚举顺序，需要固定相机序列号到语义视角，避免某次启动左右腕相机互换。
4. README 中 CAN 口和路径描述与当前脚本不完全一致，应以脚本为准：left `can3`，right `can2`，路径 `/home/elwg/Biolab/...`。
5. `hand_commands_*.csv` 是由下一帧 hand state 构造的，不是原始下发指令；训练动作语义要和 DexGraspVLA 数据构建脚本保持一致。
6. 在线推理的 normalization stats 必须和当前 step checkpoint 对应，不能用 S1 的统计量执行 S3/S5。
7. 若 S4/S6 采用固定拇指动作，仍需在顶层状态机里记录动作持续时间、触发条件、失败重试条件。

## 推荐下一步

1. 先小规模采一组 S2 和 S3，各 3-5 条 episode。
2. 用 `vla_preprocessor.py --auto-session` 生成 processed episode。
3. 检查 `aligned_data.csv` 行数、相机目录帧数、曲线图和对齐报告。
4. 明确 `rs0/rs1/zed1` 到 DexGraspVLA 训练配置的映射。
5. 再决定是否修改 launcher 加入 robot sync，之后再进入大规模分 step 采集。

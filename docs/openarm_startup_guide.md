# OpenArm 功能启动说明

## 简介

OpenArm是一个开源的7自由度类人机械臂，专为物理AI研究和接触丰富环境中的部署而设计。具有高反向驱动性和柔顺性，在提供实际载荷能力的同时，优异于安全的人机交互。

## 系统要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.8+

## 前置准备

在启动机械臂之前，请确保：

1. CAN 接口已配置（见下方）
2. 电机已初始化（参考调试说明.md 中的"新版初始化"）
3. MoveIt2 已安装（见下方安装说明）

## CAN FD 脚本一键激活

启动前需要配置 CAN 接口为 CAN-FD 模式：

```bash
cd ~/ros2_ws

# 循环配置所有 CAN 接口为 CAN-FD 模式
for can_if in can0 can1; do
    sudo ./src/openarm_can/setup/configure_socketcan.sh $can_if -fd
done
```

或手动配置：

```bash
cd ~/ros2_ws
sudo ./src/openarm_can/setup/configure_socketcan.sh can0 -fd
sudo ./src/openarm_can/setup/configure_socketcan.sh can1 -fd
```

## 安装MoveIt2（已安装可忽略）

选项1：核心包安装（~100MB，推荐）

```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit ros-humble-moveit-ros-perception
```

选项2：完整套件安装（~1.5GB）

```bash
sudo apt install ros-humble-moveit-* ros-humble-moveit-configs-utils
```

## RViz配置

启动后RViz会自动打开，需要进行以下配置：

1. 修改Fixed Frame设置：
   - 展开左侧 Global Options
   - 将 Fixed Frame 从 map 改为 openarm_link0
2. 添加机器人模型显示：
   - 点击左下角的 Add 按钮
   - 选择 RobotModel
   - 确保 Robot Description 设置为 /robot_description
3. 验证显示：
   - 机械臂模型应该正确显示在RViz中
   - 可以看到7个关节和夹爪

## 启动MoveIt

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

> 注意：如果没有连接真实机械臂必须运行 `ros2 launch openarm_bimanual_moveit_config demo.launch.py arm_type:=v10` 会显示坐标转换错误，注意添加 `arm_type:=v10 use_fake_hardware:=true`

## 验证MoveIt2功能

- 启动后应该看到MoveIt2的运动规划界面
- 可以通过拖拽交互式标记来规划机械臂运动
- 支持双臂协调控制

## 重力补偿

```bash
# 重力补偿启动(单臂)
cd src/openarm_teleop
./script/launch_grav_comp.sh <arm_side> <can_interface> <urdf_path>
# 默认参数：
# arm_side: right_arm
# can_interface: can0
# arm_type: v10
```

## 双臂力反馈启动

```bash
# right_arm（单臂、默认 CAN 总线为 can0 can2）
./script/launch_unilateral.sh right_arm can0 can2

# left_arm（默认 CAN 总线为 can1 can3）
./script/launch_bilateral.sh left_arm can1 can3
```

## 双臂遥操作（左右臂镜像跟随）

右臂作为Leader（引导臂），左臂作为Follower（跟随臂），支持双边力反馈控制。

```bash
cd ~/ros2_ws/src/openarm_teleop

# 默认配置：右臂(can0)为Leader，左臂(can1)为Follower
./script/launch_bimanual_unilateral.sh

# 交换Leader/Follower方向（左臂Leader，右臂Follower）
./script/launch_bimanual_unilateral.sh can1 can0 left_lead
```

功能说明：

- Leader（右臂）：手动拖动，感受力反馈（柔顺模式）
- Follower（左臂）：自动镜像跟随右臂运动（精确跟踪）
- 支持双边力反馈：Follower遇到阻力时，Leader会感受到反馈力

参数调整：

- Leader参数：config/leader.yaml（降低Kp/Kd使拖动更柔顺）
- Follower参数：config/follower.yaml（增加Kp/Kd使跟踪更精确）

## 双臂启动

```bash
# 轨迹控制
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10
```

参数说明：

- `arm_type` - 手臂类型（默认：v10）
- `use_fake_hardware` - 使用假硬件而不是真硬件（默认：false）
- `can_interface` - 要使用的 CAN 接口（默认：can0）
- `robot_controller` - 控制器类型：joint_trajectory_controller 或 forward_position_controller

## 夹爪控制

前置条件：启动机械臂

```bash
# 双臂启动（默认会启动所有控制器）
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10
```

控制夹爪：

```bash
cd ~/ros2_ws/src/openarm_teleop/script

# 左臂夹爪控制（张开，1秒匀速）
python3 control_gripper.py left 0.04 40 1.0 20
# 左臂夹爪控制（闭合，0.8秒更快）
python3 control_gripper.py left 0.0 40 0.8 16

# 右臂夹爪控制（张开，2秒更慢更平滑）
python3 control_gripper.py right 0.04 40 2.0 40
# 右臂夹爪控制（闭合，默认参数）
python3 control_gripper.py right 0.0

# 双臂同时控制（自定义速度/步数）
python3 control_gripper.py both 0.02 30 1.2 24   # 双臂半开，1.2秒，24步
python3 control_gripper.py both 0.0               # 双臂闭合，默认1秒20步
```

参数说明：

- `position`：目标位置（0.0=完全闭合，0.04=完全张开，中间值如0.02=半开）
- `max_effort`：最大施加力（默认40.0）
- `duration`：匀速过渡总时长（秒，默认1.0，越小越快）
- `steps`：插值步数（默认20，越多越平滑）

> 推荐：如需更快或更平滑，可自行调整 duration 和 steps 参数。

查看夹爪状态：

```bash
# 查看所有控制器
ros2 control list_controllers

# 查看关节状态
ros2 topic echo /joint_states
```

## 外骨骼遥操作

分步启动方式：

```bash
# 第1步：启动WebSocket遥操作服务，用于接收外骨骼数据
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py

# 第2步：启动外骨骼显示节点，用于可视化外骨骼运动(按需启动)
ros2 launch qnbot_teleoperator exoskeleton_display.launch.py source:=exo_command

# 第3步：启动OpenArm机器人显示节点，启用外骨骼模式(按需启动)
ros2 launch qnbot_teleoperator openarm_display.launch.py use_exoskeleton:=true

# 第4步：启动外骨骼重定向节点，将外骨骼数据映射到OpenArm机器人
ros2 launch qnbot_teleoperator exo_retargeting.launch.py robot_type:=OpenArm

# 启动硬件控制器
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller

# ros2桥接控制启动
ros2 launch qnbot_teleoperator exoskeleton_bridge.launch.py gripper_scaling_factor:=0.05
# 参数gripper_scaling_factor 夹爪缩放因子

# 手柄控制底盘+升降节点(无底盘可忽略此节点)
ros2 launch qnbot_cmd_bridge exo_cmd_bridge.launch.py
```

启动顺序重要说明：

1. 必须先启动WebSocket服务（接收外骨骼硬件数据）
2. 再启动外骨骼显示（发布外骨骼TF到exoskeleton命名空间）
3. 然后启动OpenArm显示（包含TF桥接节点，将外骨骼TF重新映射到OpenArm肩部）
4. 启动重定向节点（映射关节命令）
5. 最后启动硬件控制节点（控制实际机械臂电机）

说明：

- OpenArm使用7自由度双臂配置（v10）
- 外骨骼TF树会自动合并到OpenArm肩部位置（通过openarm_exo_tf_bridge_node）
- 关节映射配置文件：config/retargeting_OpenArm.yaml
- 第5步为可选步骤，仅在需要控制实际硬件时启动
- 硬件控制节点支持重力补偿，可通过参数配置

## 网页预设动作测试

在Ubuntu终端运行：

```bash
# 启动websocket连接
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 硬件控制启动
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller
```

浏览器打开：

```
http://visual.svtrobot.com/
```

> 输入工控机IP地址，点击连接，点右下角开始运动

## 手柄节点

```bash
ros2 launch f710_teleop f710_teleop.launch.py

# 启动后按一下A键后才能使用
```

## aimotor 阈值检测动作（可独立使用）

配置文件默认自动查找：`src/openarm_teleop/config/aimotor_trigger_move.yaml`（不写绝对路径也可用）。

一体化启动（自动带双臂 bringup）：

```bash
ros2 launch openarm_bringup openarm_bimanual_aimotor_trigger.launch.py
```

如需使用自定义配置文件，再追加参数：

```bash
ros2 launch openarm_bringup openarm_bimanual_aimotor_trigger.launch.py \
  trigger_config:=src/openarm_teleop/config/aimotor_trigger_move.yaml
```

独立启动（你已用其他方式启动机械臂时）：

```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/openarm_teleop/script/aimotor_trigger_move_bimanual.py
```

前提：

- 有 `/aimotor/position_state`（std_msgs/msg/Float64）
- 控制器命令话题正确（默认）：
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

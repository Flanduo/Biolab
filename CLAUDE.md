# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Biolab — 基于 enactic OpenArm 的双臂机械臂 + 双灵巧手二次开发项目。3 人团队，服务器 10.0.0.19 (Ubuntu 22.04, elwg 用户)。

GitHub: https://github.com/Flanduo/Biolab (owner: Flanduo)

源服务器 10.0.0.2 (用户 openarm) 上有完整环境，已迁移到本机。

### 当前硬件配置

| 设备 | 型号 | 接口 | USB 适配器 | 验证状态 |
|------|------|------|-----------|---------|
| 左臂 | OpenArm v10 (7 DOF) | can0 | PEAK PCAN-USB FD | 已验证通信正常 |
| 右臂 | OpenArm v10 (7 DOF) | can1 | PEAK PCAN-USB FD | 已连接 |
| 左灵巧手 | LinkerHand O6 (6 DOF) | can3 | PEAK PCAN-USB | 已验证连接+控制 |
| 右灵巧手 | LinkerHand O6 (6 DOF) | can2 | PEAK PCAN-USB | 已验证连接+控制 |
| 深度相机 | ZED 2i (S/N 36271931) | USB 3.0 | — | 已部署，常驻服务 :5050 |

**总自由度：36 DOF** (14 臂 + 22 手)

> 两只灵巧手已确认均为 O6 型号。代码中的 L10 配置已全部修正为 O6。

### 灵巧手验证记录

```
左手 O6 (can3): 序列号 LHO6-01-710-L-Z-1-E, 状态 [254,254,254,254,254,254] (张开)
右手 O6 (can2): 序列号 LHO6-00-000-0-0-1-X, 状态 [254,254,254,254,254,254] (张开)
SDK 版本: 3.1.0
```

### 机械臂验证记录

```
ros2 launch openarm_bringup openarm.launch.py
- robot_state_publisher: 加载 openarm_link0~link7, openarm_hand, openarm_hand_tcp 等
- ros2_control_node: openarm_hardware_interface 加载成功
- CAN=can0, arm_prefix=, hand=enabled, can_fd=enabled
- 8 packages 编译通过, 0 error, 仅有 warning (unused parameter)
```

## Environment Setup

### 已安装环境

| 组件 | 版本/路径 | 状态 |
|------|----------|------|
| Ubuntu | 22.04 LTS (jammy) | apt 源已换清华镜像 |
| Git | 系统自带 | elwg user.name=Flanduo, email=2089767109@qq.com |
| SSH key | ed25519 | 已生成，公钥待添加到 GitHub |
| Miniconda | 26.1.1 ~/miniconda3 | 清华镜像已配 |
| Conda 环境 | ~/Biolab/conda_envs/ros_env | 从 10.0.0.2 迁移，已 conda-unpack |
| ROS2 Humble | ros-humble-desktop + moveit + ros2-control | 完整安装 |
| LinkerHand SDK | v3.1.0 ~/Biolab/linkerhand-sdk | 依赖已安装，可正常 import |
| ZED SDK | 5.2.3 /usr/local/zed/ | pyzed 已装到系统 Python 3.10 |
| FFmpeg | 4.4.2 | 录制视频 H.264 转码用 |

### 环境激活

```bash
# Conda 环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate /home/elwg/Biolab/conda_envs/ros_env

# ROS2 环境
source /opt/ros/humble/setup.bash
source /home/elwg/Biolab/ros2_ws/install/setup.bash

# 快捷编译
alias cb='cd /home/elwg/Biolab/ros2_ws && colcon build --symlink-install'
```

### CAN 设备启动 (每次重新插拔后需要)

```bash
# 机械臂 (需要 sudo)
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# 灵巧手 (需要 sudo)
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000

# 检查状态
ip link show | grep can
```

## Build Commands

```bash
# ROS2 工作空间编译
cd /home/elwg/Biolab/ros2_ws
colcon build --symlink-install

# 编译单个包
colcon build --packages-select openarm_description --symlink-install

# 编译依赖缺失时
sudo apt install -f
colcon build --symlink-install
```

## Key Directories

| 目录 | 用途 |
|------|------|
| `ros2_ws/src/` | ROS2 包源码 (openarm_can, openarm_description, openarm_bringup, openarm_hardware, openarm_teleop, qnbot_teleoperator, chassis_control) |
| `openarm_demo/` | 双臂控制主程序 (Python WebApp + IK + 可视化)，有独立的 [CLAUDE.md](openarm_demo/CLAUDE.md) |
| `linkerhand-sdk/` | LinkerHand 灵巧手 Python SDK (v3.1.0)，不支持 pip install，需通过 PYTHONPATH 或 linkerhand_control.py 封装使用 |
| `ZEDProject/` | ZED 2i 深度相机服务 (capture_server.py)，有独立的 [CLAUDE.md](ZEDProject/CLAUDE.md) |
| `our_modules/` | 自定义模块 (control/, perception/, planning/)，perception 已有 camera_client.py |
| `configs/` | 全局配置文件 (default.yaml) |
| `scripts/` | 部署和环境脚本 (deploy.sh, pack_conda.sh, setup_env.sh) |
| `conda_envs/ros_env/` | Conda 环境 (从 10.0.0.2 打包迁移) |

## ROS2 Launch

```bash
source /opt/ros/humble/setup.bash
source ~/Biolab/ros2_ws/install/setup.bash

# 单臂 (默认 can0)
ros2 launch openarm_bringup openarm.launch.py

# 双臂 (can0 + can1)
ros2 launch openarm_bringup openarm.bimanual.launch.py

# 灵巧手按钮控制 (手柄 Button C 握拳, Button D 张开)
ros2 launch qnbot_teleoperator linkerhand_button_controller.launch.py

# MoveIt 运动规划 + RViz
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

> 注意: 服务器无图形界面，RViz 会报 xcb 错误。如需可视化请使用 WebApp 或转 发 X11。

## 升降机 (Chassis)

底盘升降模组，由 `chassis_control` ROS2 包控制，提供升降电机位置反馈。

```bash
# 一键启动 (rosbridge + lift_state_node)
bash ~/Biolab/scripts/start_lift.sh

# 手动启动
source /opt/ros/humble/setup.bash
source ~/Biolab/ros2_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml  # 终端1
ros2 run chassis_control lift_state_node                      # 终端2

# 一键启动全部底盘节点 (升降 + 底盘控制 + 轮毂电机)
ros2 launch chassis_control svtrobo_bringup.launch.py
```

| 节点 | 功能 |
|------|------|
| `lift_state_node` | 升降模组电机控制与位置反馈 |
| `chassis_control` | 底盘控制 |
| `zlac8015d_rpm_node` | 轮毂电机转速反馈 |

> 前提: `chassis_control` 包需已编译 (`colcon build --packages-select chassis_control`)

## LinkerHand 灵巧手控制

SDK 位于 `linkerhand-sdk/`，通过 CAN (SocketCAN) 通信。

### Python 直接控制

```python
# 必须在 openarm_demo 目录下运行（utils 模块依赖）
from utils.linkerhand_control import LinkerHandControl

# 左手 O6 (can2)
left = LinkerHandControl("left", "O6", can="can2")
left.connect()                    # 连接
left.move([128] * 6)              # 6个关节，0-255 (0=弯曲, 255=张开)
state = left.get_state()          # 读取当前状态
left.set_speed([150] * 6)         # 设置速度 (10-255)
left.disconnect()

# 右手 O6 (can3) — 同左手指令
right = LinkerHandControl("right", "O6", can="can3")
right.connect()
```

### SDK 路径配置

`openarm_demo/utils/linkerhand_control.py` 中的 `SDK_PATH` 已改为本机路径:
```python
SDK_PATH = "/home/elwg/Biolab/linkerhand-sdk"
```

`ros2_ws/src/qnbot_teleoperator/qnbot_teleoperator/linkerhand_button_controller.py` 中的路径也已更新:
```python
_ROS_ENV_SITE_PACKAGES = '/home/elwg/Biolab/conda_envs/ros_env/lib/python3.10/site-packages'
LINKERHAND_SDK_PATH = '/home/elwg/Biolab/linkerhand-sdk'
```

## WebApp (openarm_demo)

详见 `openarm_demo/CLAUDE.md`。启动方式：
```bash
cd ~/Biolab/openarm_demo/webapp
conda activate elwgdemo
./start.sh
```

Flask 后端 :5000，Viser 3D 可视化 :8080。

> 注意: `elwgdemo` conda 环境尚未在本机创建。

## ZED 相机服务

详见 `ZEDProject/CLAUDE.md`。

```bash
# 启动 (默认 2K 15FPS)
~/Biolab/ZEDProject/start_capture.sh

# 指定分辨率
~/Biolab/ZEDProject/start_capture.sh --resolution 1080
```

### Python 客户端 (全流程脚本用)

```python
import sys; sys.path.insert(0, '/home/elwg/Biolab')
from our_modules.perception import CameraClient

cam = CameraClient()
cam.start_recording()                    # 开始录制
result = cam.capture("target")           # 拍照 (彩色+深度)
intrinsics = cam.get_intrinsics()        # 相机内参
cam.stop_recording()                     # 停止，自动转码 H.264
```

### 相机服务 API 速查

| 方法 | 路由 | 功能 |
|------|------|------|
| GET | /status | 相机状态 |
| POST | /capture | 拍照 (object=名称, transfer=SCP到主控机) |
| GET | /intrinsics | 内参矩阵 |
| GET | /stream | MJPEG RGB 实时流 |
| POST | /recording/start | 开始录制 |
| POST | /recording/stop | 停止录制 + H.264 转码 |

> 约束: GTX 1080 Ti (Pascal) 不支持 NEURAL 深度模式，使用 ULTRA。相机随服务常驻不释放。

## sudo 权限

elwg 用户在 sudo 组但需要密码。需要 sudo 的操作（apt、CAN 配置、ROS2 安装等）需要用户在终端手动执行。管理员用户 wyf 负责系统级维护。不要修改 `/home/wyf/` 下的任何内容。

## Git

- 远程: HTTPS (`https://github.com/Flanduo/Biolab.git`)
- elwg 的 SSH key 已生成 (`~/.ssh/id_ed25519`)，**公钥尚未添加到 GitHub**
- `.git` 目录由 wyf 创建，已配置 `git config --global --add safe.directory /home/elwg/Biolab`
- SSH 免密已配: `openarm@10.0.0.2`

## 源服务器

10.0.0.2 (用户 openarm)，已配免密 SSH。用于同步代码和环境。可用的同步命令:
```bash
# 同步 ROS2 源码
rsync -avz --exclude='build' --exclude='install' --exclude='log' \
  openarm@10.0.0.2:/home/openarm/ros2_ws/src/ ~/Biolab/ros2_ws/src/

# 同步 openarm_demo
rsync -avz --exclude='.git' \
  openarm@10.0.0.2:/home/openarm/openarm_demo/ ~/Biolab/openarm_demo/
```

## 已完成的 L10→O6 修正

右手灵巧手从 L10 (20 DOF) 修正为 O6 (11 DOF)，修改的文件：

| 文件 | 修改内容 |
|------|---------|
| `openarm_demo/webapp/backend/config.py` | ROBOT_DESCRIPTION 右手 → O6, 手部动作配置 → O6 11关节 |
| `openarm_demo/webapp/backend/kinematics/config.py` | 右手关节名/限位/姿态全部改为 O6, TOTAL_JOINTS=36 |
| `openarm_demo/webapp/backend/services/hardware_service.py` | 右手关节数 20→11, 变量名 L10→O6 |
| `openarm_demo/viser_ros_control.py` | 注释更新 |
| `openarm_demo/CLAUDE.md` | 文档更新 |
| `ros2_ws/.../linkerhand_button_controller.py` | SDK路径、ROS_ENV路径、默认参数 L6→O6 |
| `ros2_ws/.../linkerhand_button_controller.launch.py` | 默认参数 L6→O6 |

## 待办事项

- [ ] elwg SSH 公钥添加到 GitHub Settings → SSH Keys
- [ ] 创建 `elwgdemo` conda 环境用于 WebApp
- [ ] `openarm_demo/visualize_openarm.py` 中仍有 L10 右手可视化代码，需要适配 O6
- [ ] `openarm_demo/viser_ros_control.py` 中右手控制逻辑需要适配 O6
- [ ] `our_modules/` 模块开发 (control/perception/planning)
- [ ] 相机已部署，perception 模块 camera_client.py 已封装
- [ ] SSH 免密到 10.0.0.18 (主控机) 待配置，SCP 传输功能暂不可用
- [ ] 对接主控机 GSA 检测 + FoundationPose 位姿估计 API

# Biolab 硬件调试启动指南

## 网络拓扑

```
本机 (10.0.0.121)  ──浏览器访问──> http://10.0.0.19:8082
                                         │
服务器 (10.0.0.19)                        │
  ├─ 终端1: ROS2 双臂驱动 (can0/can1)     │
  ├─ 终端2: rosbridge (:9090)  <──────── viser 通过 WebSocket 桥接
  ├─ 终端3: viser 3D 控制台 (:8082) <─────┘
  ├─ 机械臂 x2  (can0, can1)
  └─ 灵巧手 x2  (can2, can3)
```

## 一、启动前检查

### 检查 CAN 设备（4个都应该是 UP）

```bash
ip link show | grep can
```

如果 DOWN 或不存在，执行：

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000
```

### 检查 USB 设备

```bash
lsusb | grep -i peak
# 应看到 4 个 PEAK 设备:
#   2x PEAK System PCAN-USB FD  (机械臂)
#   2x PEAK System PCAN-USB     (灵巧手)
```

## 二、启动步骤（3 个终端）

### 终端 1: ROS2 双臂驱动

```bash
source /opt/ros/humble/setup.bash
source ~/Biolab/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

> 验证: 看到 `openarm_hardware_interface` 加载成功，`CAN=can0, can_fd=enabled`

### 终端 2: rosbridge（Web 桥接）

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> 验证: 看到 `WebSocket server listening on port 9090`
>
> 如未安装: `sudo apt install -y ros-humble-rosbridge-suite`

### 终端 3: viser 3D 可视化控制

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate /home/elwg/Biolab/conda_envs/ros_env
cd ~/Biolab/openarm_demo
python3 viser_ros_control.py
```

> 首次运行会构建 Web 客户端，稍等片刻。完成后显示 `http://0.0.0.0:8082`

## 三、本机浏览器访问

打开 **http://10.0.0.19:8082**

## 四、真机操作步骤（按顺序）

### 步骤 1: 连接 ROS

在网页左侧面板找到 **"ROS 连接"**：
- 服务器: `localhost`
- 端口: `9090`
- 点击 **"连接"** → 状态变为 "已连接"

### 步骤 2: 连接灵巧手

找到 **"灵巧手连接"**：
- 点击 **"连接左手 O6"** → 状态显示 O6✓
- 点击 **"连接右手 O6"** → 状态显示 O6✓
- 可调节左右手速度（默认 80，范围 0-255）

### 步骤 3: 同步真实位置（重要！）

找到 **"控制模式"**：
- 点击 **"同步真实位置"** 按钮
- 这会读取机械臂当前实际关节角度并更新滑块
- **不先同步直接动滑块可能导致机械臂突然跳到目标位置！**

### 步骤 4: 开启真机模式

- 勾选 **"真机模式"** 复选框
- 此时滑块变化会直接驱动真实机械臂
- 建议先将 **"机械臂速度"** 滑块调低到 0.05~0.1 rad/s

### 步骤 5: 控制机械臂运动

- 拖动 **左臂 joint1~joint7** 滑块 → 机械臂左臂跟随运动
- 拖动 **右臂 joint1~joint7** 滑块 → 机械臂右臂跟随运动
- 运动有平滑插值，不会突变

### 步骤 6: 控制灵巧手

在灵巧手面板中：
- 6 个滑块独立控制每个手指关节（0=弯曲, 255=张开）
- **预设按钮**：张开 / 握拳 / OK / 点赞
- 灵巧手在真机模式下会实时跟随滑块

### 预设动作

- **"启动姿态"**: 机械臂运动到预设启动位置（需真机模式）
- **"复位"**: 机械臂回到零位（需真机模式）
- **"急停"**: 立即停止所有运动，切回仿真模式

### 姿态记录

- 输入名称 → 点击 **"记录姿态"** 保存当前关节角度
- 可回放已保存的姿态序列

## 五、安全注意事项

1. **首次测试建议先用仿真模式**熟悉界面操作
2. **开启真机前务必先同步真实位置**，避免机械臂跳变
3. **速度先调低** (0.05~0.1 rad/s)，确认无误后再加快
4. 确保机械臂周围 **无障碍物、无人员靠近**
5. 手边保持 **急停按钮** 可用（网页上的急停或硬件急停）
6. 灵巧手测试时先用手托住，防止夹伤

## 六、关节限位参考

### 左臂 (弧度)

| 关节 | 范围 |
|------|------|
| joint1 | -0.8 ~ 1.0 |
| joint2 | -1.65 ~ 0 |
| joint3 | -1.7 ~ 1.7 |
| joint4 | -0.9 ~ 1.8 |
| joint5 | -1.76 ~ 1.76 |
| joint6 | -0.8 ~ 0.8 |
| joint7 | -1.7 ~ 1.7 |

### 右臂 (弧度)

| 关节 | 范围 |
|------|------|
| joint1 | -1.0 ~ 1.37 |
| joint2 | 0 ~ 1.93 |
| joint3 | -1.7 ~ 1.7 |
| joint4 | 0 ~ 2.0 |
| joint5 | -1.7 ~ 1.7 |
| joint6 | -1.0 ~ 1.0 |
| joint7 | -1.7 ~ 1.7 |

## 七、CAN 端口分配

| CAN 口 | 设备 | USB 适配器 | bitrate |
|--------|------|-----------|---------|
| can0 | 左臂 OpenArm v10 | PEAK PCAN-USB FD | 1000000 |
| can1 | 右臂 OpenArm v10 | PEAK PCAN-USB FD | 1000000 |
| can2 | 左灵巧手 O6 | PEAK PCAN-USB | 1000000 |
| can3 | 右灵巧手 O6 | PEAK PCAN-USB | 1000000 |

## 八、常见问题

### Q: 浏览器打不开 8082
检查防火墙: `sudo ufw status`，如启用需放行 8082/9090 端口

### Q: CAN 设备 No such device
USB 线未插好，重新插拔适配器后重新 `sudo ip link set canX up`

### Q: 机械臂不响应
1. 终端1是否正常运行
2. 终端2 rosbridge 是否运行
3. 网页是否已连接 ROS (状态=已连接)
4. 是否已勾选"真机模式"

### Q: 灵巧手连接失败
1. can2/can3 是否 UP
2. 重新插拔 USB-CAN 适配器
3. 重启 viser_ros_control.py

### Q: 机械臂突然跳动
没有先同步真实位置就开启了真机模式。点击急停，然后先同步再操作。

## 九、快速停止

```bash
# 各终端按 Ctrl+C 停止对应服务

# 关闭 CAN
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can2 down
sudo ip link set can3 down
```

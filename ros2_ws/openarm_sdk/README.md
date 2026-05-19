# OpenArm SDK

用于 OpenArm 机械臂控制的高级 Python SDK。

## 安装

```bash
# 1. 底层初始化（只需一次）
cd ~/ros2_ws && ./init_motors.sh

# 2. 构建 openarm_can Python 绑定
cd ~/ros2_ws
colcon build --packages-select openarm_can
cd src/openarm_can/python
pip3 install --no-build-isolation -e .

# 3. 安装 SDK
cd ~/ros2_ws/openarm_sdk
pip install -e .
```

## 快速开始

**运行示例**（必须在 `openarm_sdk` 目录下）：

```bash
cd ~/ros2_ws/openarm_sdk
python3 examples/basic_control.py
```

**代码示例**：

```python
from openarm_sdk import OpenArmSDK, ControlMode

sdk = OpenArmSDK(can_interface="can0", enable_can_fd=True)
sdk.init_arm_motors()
sdk.set_control_mode(ControlMode.GRAVITY_COMPENSATION)
sdk.start_control_loop()
```

## 功能

- ✅ 重力补偿控制
- ✅ 自定义动力学计算（回调函数）
- ✅ 配置管理（YAML）
- ✅ 控制循环框架（100Hz）
- 🚧 双边控制（框架就绪）

## 示例

- `examples/basic_control.py` - 基础控制
- `examples/gravity_compensation.py` - 重力补偿
- `examples/config_example.py` - 配置加载

详细文档：
- [QUICKSTART.md](QUICKSTART.md) - 快速开发指南

## 系统要求

- Python 3.8+
- Linux + SocketCAN

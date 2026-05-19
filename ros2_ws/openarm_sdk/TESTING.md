# OpenArm SDK 测试指南

## 前提条件

### 1. 构建 openarm_can Python 绑定

```bash
# 构建 C++ 库
cd ~/ros2_ws
colcon build --packages-select openarm_can

# 构建 Python 绑定
cd src/openarm_can/python
pip3 install --no-build-isolation -e .

# 测试导入
python3 -c "from openarm.can import OpenArm; print('OK')"
```

### 2. 安装 SDK

```bash
cd ~/ros2_ws/openarm_sdk
pip3 install -e .
python3 -c "from openarm_sdk import OpenArmSDK; print('OK')"
```

## 测试步骤

### 1. 配置加载测试（无需硬件）

```bash
cd ~/ros2_ws/openarm_sdk
python3 examples/config_example.py
```

### 2. 基础控制测试（需要硬件）

```bash
# 确保已初始化
cd ~/ros2_ws && ./init_motors.sh

# 运行测试（注意：必须在 openarm_sdk 目录下运行）
cd ~/ros2_ws/openarm_sdk
python3 examples/basic_control.py
```

### 3. 重力补偿测试（需要硬件）

```bash
# 注意：必须在 openarm_sdk 目录下运行
cd ~/ros2_ws/openarm_sdk
python3 examples/gravity_compensation.py
# 按 Ctrl+C 停止
```

## 常见问题

**导入错误**: 确保已构建并安装 `openarm_can` Python 绑定

**CAN 接口不存在**: 检查硬件连接，运行 `ip link show can0`

**权限错误**: `sudo usermod -a -G dialout $USER` 然后重新登录

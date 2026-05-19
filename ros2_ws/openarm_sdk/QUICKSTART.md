# OpenArm SDK 快速开发指南

## 前提条件

```bash
# 1. 底层初始化（只需一次）
cd ~/ros2_ws && ./init_motors.sh

# 2. 安装 SDK
cd ~/ros2_ws/openarm_sdk && pip install -e .
```

## 快速开始

**重要**: 运行示例时，必须在 `openarm_sdk` 目录下执行：

```bash
cd ~/ros2_ws/openarm_sdk
python3 examples/basic_control.py
```

或者使用启动脚本：

```bash
cd ~/ros2_ws/openarm_sdk
chmod +x run_example.sh
./run_example.sh basic_control
```

### 重力补偿模式

```python
from openarm_sdk import OpenArmSDK, ControlMode

sdk = OpenArmSDK(can_interface="can0", enable_can_fd=True)
sdk.init_arm_motors()

sdk.set_control_mode(ControlMode.GRAVITY_COMPENSATION)
sdk.start_control_loop()
# 按 Ctrl+C 停止
```

### 自定义动力学计算

```python
from openarm_sdk import OpenArmSDK, ControlMode, CallbackDynamics

def calculate_gravity(joint_positions):
    # 调用您的动力学库（KDL、C++ 绑定等）
    return gravity_torques  # 返回扭矩列表

dynamics = CallbackDynamics(gravity_callback=calculate_gravity)
sdk = OpenArmSDK(can_interface="can0", enable_can_fd=True)
sdk.init_arm_motors()
sdk.set_control_mode(ControlMode.GRAVITY_COMPENSATION, dynamics=dynamics)
sdk.start_control_loop()
```

### 状态监控

```python
from openarm_sdk import OpenArmSDK

sdk = OpenArmSDK(can_interface="can0", enable_can_fd=True)
sdk.init_arm_motors()
sdk.refresh_all()
sdk.recv_all()

arm = sdk.get_arm()
for i, motor in enumerate(arm.get_motors()):
    print(f"电机 {i+1}: 位置={motor.get_position():.3f}, "
          f"速度={motor.get_velocity():.3f}, 扭矩={motor.get_torque():.3f}")
```

### 自定义控制算法

```python
from openarm_sdk import OpenArmSDK, ControlLoop
from openarm.can import MITParam

class MyControl(ControlLoop):
    def step(self):
        arm = self.sdk.get_arm()
        motors = arm.get_motors()
        
        cmds = [MITParam(0, 0, 0, 0, your_algorithm(motor)) 
                for motor in motors]
        arm.mit_control_all(cmds)
        self.sdk.recv_all()

sdk = OpenArmSDK(can_interface="can0", enable_can_fd=True)
sdk.init_arm_motors()
control = MyControl(sdk)
control.start()
```

## 核心 API

- `enable_all()` / `disable_all()` - 使能/禁用电机
- `set_zero_all()` - 设置零位
- `refresh_all()` / `recv_all()` - 刷新状态
- `set_control_mode(ControlMode.GRAVITY_COMPENSATION)` - 设置控制模式
- `start_control_loop()` / `stop_control_loop()` - 启动/停止控制循环

更多示例见 `examples/` 目录。

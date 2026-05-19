# openarm_arm_teleop

OpenArm 真机手柄末端控制说明（简版）。

## 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_arm_teleop
source install/setup.bash
```

## 真机启动（末端控制）

终端1：启动双臂真机

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10
```

终端2：启动 MoveIt（提供 IK）

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py arm_type:=v10
```

终端3：启动手柄末端控制

```bash
ros2 run openarm_arm_teleop ee_teleop_node
```

也可以用统一 launch 切模式：

```bash
# 关节控制（旧模式）
ros2 launch openarm_arm_teleop openarm_arm_teleop.launch.py mode:=joint

# 末端控制（IK 模式）
ros2 launch openarm_arm_teleop openarm_arm_teleop.launch.py mode:=ee
```

## 按键映射（ee_teleop）

- 左摇杆前后：末端 `x`
- 左摇杆左右：末端 `y`
- 右摇杆上下：末端 `z`
- `X`：只控左臂；`Y`：只控右臂；`X+Y`：双臂同步
- `LT`：夹爪闭合；`RT`：夹爪张开
- 按住 `LB`：精调（低速）
- `RB`：回零（当前模式下手臂 7 关节回到 0）

## 快速检查

```bash
ros2 service list | grep compute_ik
ros2 control list_controllers
```

## 抖动/乱跑时建议参数

如果末端到某些位置出现抖动或不跟手，先降低速度并限制单步关节跳变：

```bash
ros2 run openarm_arm_teleop ee_teleop_node --ros-args \
  -p device_path:=/dev/input/js0 \
  -p linear_speed:=0.08 \
  -p fine_scale:=0.25 \
  -p axis_smoothing_alpha:=0.20 \
  -p max_joint_step:=0.20 \
  -p ik_seed_tolerance:=0.30
```

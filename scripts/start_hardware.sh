#!/bin/bash
# Biolab 硬件调试一键启动脚本
# 自动配置 CAN + 启动 ROS2 双臂驱动 + rosbridge + viser 控制台
#
# 用法: bash scripts/start_hardware.sh
# 停止: bash scripts/stop_hardware.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CAN_SETUP="$PROJECT_DIR/ros2_ws/src/openarm_can/setup/configure_socketcan"

echo "========================================="
echo "  Biolab 硬件调试一键启动"
echo "========================================="

# ---- Step 1: 配置 CAN 设备 ----
echo ""
echo "[1/5] 配置 CAN 设备..."

# 先关闭所有 CAN 接口
for iface in can0 can1 can2 can3; do
    sudo ip link set "$iface" down 2>/dev/null || true
done

# 机械臂: CAN-FD 模式 (can0, can1)
if [ -f "$CAN_SETUP" ]; then
    echo "  配置 can0 (左臂) CAN-FD..."
    sudo "$CAN_SETUP" can0 -fd
    echo "  配置 can1 (右臂) CAN-FD..."
    sudo "$CAN_SETUP" can1 -fd
else
    echo "  ⚠️  configure_socketcan 脚本未找到，使用手动配置"
    sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
    sudo ip link set can0 up
    sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
    sudo ip link set can1 up
fi

# 灵巧手: CAN 2.0 (can2, can3)
echo "  配置 can2 (左灵巧手 O6) CAN 2.0..."
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up
echo "  配置 can3 (右灵巧手 O6) CAN 2.0..."
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can3 up

# 验证
CAN_COUNT=$(ip link show | grep -c "can[0-3].*UP" || true)
if [ "$CAN_COUNT" -lt 4 ]; then
    echo "  ❌ 只有 $CAN_COUNT/4 个 CAN 设备 UP，请检查 USB 连接"
    exit 1
fi
echo "  ✅ 4/4 CAN 设备正常 (can0~can3)"

# ---- Step 2: 加载环境 ----
echo ""
echo "[2/5] 加载环境..."
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate "$PROJECT_DIR/conda_envs/ros_env"
echo "  ✅ ROS2 + conda 环境已加载"

# ---- Step 3: 停止旧会话 ----
SESSION="biolab_hardware"
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# ---- Step 4: 启动 ROS2 双臂驱动 ----
echo ""
echo "[3/5] 启动 ROS2 双臂驱动 (CAN-FD, forward_position_controller)..."
tmux new-session -d -s "$SESSION" -n "ros2_arm"
tmux send-keys -t "$SESSION:ros2_arm" \
    "source /opt/ros/humble/setup.bash && \
     source $PROJECT_DIR/ros2_ws/install/setup.bash && \
     ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller" Enter

# 等待 ROS2 驱动加载硬件
echo "  等待硬件加载 (5秒)..."
sleep 5

# ---- Step 5: 启动 rosbridge ----
echo ""
echo "[4/5] 启动 rosbridge..."
tmux new-window -t "$SESSION" -n "rosbridge"
tmux send-keys -t "$SESSION:rosbridge" \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml" Enter

echo "  等待 rosbridge 启动 (3秒)..."
sleep 3

# ---- Step 6: 启动 viser ----
echo ""
echo "[5/5] 启动 viser 3D 控制台..."
tmux new-window -t "$SESSION" -n "viser"
tmux send-keys -t "$SESSION:viser" \
    "source ~/miniconda3/etc/profile.d/conda.sh && \
     conda activate $PROJECT_DIR/conda_envs/ros_env && \
     cd $PROJECT_DIR/openarm_demo && \
     python3 viser_ros_control.py" Enter

echo ""
echo "========================================="
echo "  ✅ 全部服务已启动!"
echo "========================================="
echo ""
echo "  tmux 会话:  $SESSION"
echo "  查看终端:   tmux attach -t $SESSION"
echo "  切换窗口:   Ctrl+B 然后按 0/1/2"
echo "  退出查看:   Ctrl+B 然后按 d"
echo ""
echo "  窗口 0: ROS2 双臂驱动"
echo "  窗口 1: rosbridge :9090"
echo "  窗口 2: viser 控制台 :8082"
echo ""
echo "  浏览器打开: http://10.0.0.19:8082"
echo ""
echo "  停止所有服务: tmux kill-session -t $SESSION"
echo "  或执行: bash scripts/stop_hardware.sh"
echo "========================================="

#!/bin/bash
# Biolab 升降机启动脚本
# 启动 rosbridge (WebSocket) + 升降机状态反馈节点
#
# 用法: bash scripts/start_lift.sh
# 前提: ROS2 环境已加载 (source /opt/ros/humble/setup.bash + ros2_ws/install/setup.bash)
# 停止: Ctrl+C 或 bash scripts/stop_hardware.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 加载环境
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"

SESSION="biolab_lift"

echo "========================================="
echo "  Biolab 升降机启动"
echo "========================================="

# 停止旧会话
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# 窗口 0: rosbridge WebSocket (端口 9090)
tmux new-session -d -s "$SESSION" -n "rosbridge"
tmux send-keys -t "$SESSION:rosbridge" \
    "source /opt/ros/humble/setup.bash && \
     source $PROJECT_DIR/ros2_ws/install/setup.bash && \
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml" Enter

echo "  等待 rosbridge 启动 (3秒)..."
sleep 3

# 窗口 1: 升降机状态反馈节点
tmux new-window -t "$SESSION" -n "lift"
tmux send-keys -t "$SESSION:lift" \
    "source /opt/ros/humble/setup.bash && \
     source $PROJECT_DIR/ros2_ws/install/setup.bash && \
     ros2 run chassis_control lift_state_node" Enter

echo ""
echo "========================================="
echo "  升降机服务已启动!"
echo "========================================="
echo ""
echo "  tmux 会话:  $SESSION"
echo "  查看终端:   tmux attach -t $SESSION"
echo "  窗口 0: rosbridge :9090"
echo "  窗口 1: lift_state_node"
echo ""
echo "  停止服务: tmux kill-session -t $SESSION"
echo "========================================="

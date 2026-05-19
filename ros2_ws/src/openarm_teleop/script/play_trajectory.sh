#!/bin/bash
#
# OpenArm关节轨迹播放脚本
# 使用方法: ./play_trajectory.sh <input_file> [options]
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ -z "$1" ]; then
    echo "错误: 请指定输入文件"
    echo "使用方法: $0 <input_file> [--loop] [--speed <factor>]"
    exit 1
fi

INPUT_FILE="$1"
shift  # 移除第一个参数，剩下的传递给Python脚本

if [ ! -f "$INPUT_FILE" ]; then
    echo "错误: 文件不存在: $INPUT_FILE"
    exit 1
fi

echo "=========================================="
echo "OpenArm关节轨迹播放"
echo "=========================================="
echo "输入文件: $INPUT_FILE"
echo "参数: $@"
echo ""
echo "提示:"
echo "  - 按 Ctrl+C 停止播放"
echo "  - 或使用服务停止: ros2 service call /stop_playback std_srvs/srv/SetBool '{data: true}'"
echo "=========================================="
echo ""

# 切换到工作空间
cd "$WS_DIR"

# Source ROS 2
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# 运行播放节点
python3 "$SCRIPT_DIR/play_joint_trajectory.py" --input "$INPUT_FILE" "$@"

echo ""
echo "✅ 播放完成"


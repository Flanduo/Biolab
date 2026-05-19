#!/bin/bash
#
# OpenArm关节轨迹录制脚本
# 使用方法: ./record_trajectory.sh [output_file]
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 默认输出文件
if [ -z "$1" ]; then
    OUTPUT_FILE="trajectory_$(date +%Y%m%d_%H%M%S).json"
else
    OUTPUT_FILE="$1"
fi

# 确保输出文件有.json扩展名
if [[ ! "$OUTPUT_FILE" == *.json ]]; then
    OUTPUT_FILE="${OUTPUT_FILE}.json"
fi

echo "=========================================="
echo "OpenArm关节轨迹录制"
echo "=========================================="
echo "输出文件: $OUTPUT_FILE"
echo ""
echo "提示:"
echo "  - 按 Ctrl+C 停止录制并保存"
echo "  - 或使用服务停止: ros2 service call /stop_recording std_srvs/srv/SetBool '{data: true}'"
echo "=========================================="
echo ""

# 切换到工作空间
cd "$WS_DIR"

# Source ROS 2
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# 运行录制节点
python3 "$SCRIPT_DIR/record_joint_trajectory.py" --output "$OUTPUT_FILE"

echo ""
echo "✅ 录制完成，文件保存在: $OUTPUT_FILE"


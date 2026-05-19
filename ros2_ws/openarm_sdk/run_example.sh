#!/bin/bash
# OpenArm SDK 示例运行脚本
# 自动检测正确的路径并运行示例

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_NAME="$1"

if [ -z "$EXAMPLE_NAME" ]; then
    echo "用法: $0 <example_name>"
    echo ""
    echo "可用示例:"
    echo "  basic_control        - 基础控制示例"
    echo "  gravity_compensation - 重力补偿示例"
    echo "  config_example       - 配置加载示例"
    echo ""
    echo "示例:"
    echo "  $0 basic_control"
    exit 1
fi

EXAMPLE_FILE="$SCRIPT_DIR/examples/${EXAMPLE_NAME}.py"

if [ ! -f "$EXAMPLE_FILE" ]; then
    echo "❌ 错误: 找不到示例文件 $EXAMPLE_FILE"
    echo ""
    echo "请确保在正确的目录下运行:"
    echo "  cd ~/ros2_ws/openarm_sdk"
    echo "  $0 $EXAMPLE_NAME"
    exit 1
fi

# 切换到正确的目录
cd "$SCRIPT_DIR"

# 运行示例
python3 "$EXAMPLE_FILE" "${@:2}"


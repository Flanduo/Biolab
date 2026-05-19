#!/bin/bash
#
# 检查和构建 openarm_can Python 绑定的脚本
#

set -e

echo "=========================================="
echo "检查 openarm_can Python 绑定"
echo "=========================================="
echo ""

# 工作空间根目录
WS_DIR="${ROS2_WS:-$HOME/ros2_ws}"
CAN_PYTHON_DIR="$WS_DIR/src/openarm_can/python"

if [ ! -d "$WS_DIR" ]; then
    echo "❌ 错误: 工作空间目录不存在: $WS_DIR"
    exit 1
fi

if [ ! -d "$CAN_PYTHON_DIR" ]; then
    echo "❌ 错误: openarm_can Python 目录不存在: $CAN_PYTHON_DIR"
    exit 1
fi

# 检查 Python 环境
echo "[1/5] 检查 Python 环境..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 未安装"
    exit 1
fi
python_version=$(python3 --version)
echo "  ✓ $python_version"

# 检查依赖
echo ""
echo "[2/5] 检查依赖包..."
missing_deps=()
if ! python3 -c "import yaml" 2>/dev/null; then
    missing_deps+=("pyyaml")
fi
if ! python3 -c "import numpy" 2>/dev/null; then
    missing_deps+=("numpy")
fi

if [ ${#missing_deps[@]} -gt 0 ]; then
    echo "  ⚠️  缺少依赖: ${missing_deps[*]}"
    echo "  安装命令: pip3 install ${missing_deps[*]}"
    read -p "  是否现在安装？(y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip3 install "${missing_deps[@]}"
    else
        echo "  请手动安装后再继续"
        exit 1
    fi
else
    echo "  ✓ 所有依赖已安装"
fi

# 检查 openarm_can C++ 库是否已构建
echo ""
echo "[3/5] 检查 openarm_can C++ 库..."
CAN_BUILD_DIR="$WS_DIR/src/openarm_can/build"
if [ ! -d "$CAN_BUILD_DIR" ] || [ ! -f "$CAN_BUILD_DIR/libopenarm_can.so" ]; then
    echo "  ⚠️  C++ 库未构建，需要先构建"
    echo "  构建方式 1: colcon build --packages-select openarm_can"
    echo "  构建方式 2: cd src/openarm_can && mkdir -p build && cd build && cmake .. && make"
    read -p "  是否现在构建 C++ 库？(y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "  正在构建 C++ 库..."
        cd "$WS_DIR"
        colcon build --packages-select openarm_can
    else
        echo "  请先构建 C++ 库"
        exit 1
    fi
else
    echo "  ✓ C++ 库已构建"
fi

# 尝试导入 openarm_can
echo ""
echo "[4/5] 测试 openarm_can Python 绑定导入..."

# 添加路径
export PYTHONPATH="$CAN_PYTHON_DIR:$PYTHONPATH"

if python3 -c "from openarm.can import OpenArm" 2>/dev/null; then
    echo "  ✓ openarm_can Python 绑定可用"
else
    echo "  ⚠️  openarm_can Python 绑定未找到，需要构建"
    echo ""
    echo "  正在构建 Python 绑定..."
    cd "$CAN_PYTHON_DIR"
    
    # 检查构建脚本
    if [ ! -f "build.sh" ]; then
        echo "  ❌ 错误: build.sh 不存在"
        exit 1
    fi
    
    chmod +x build.sh
    ./build.sh
    
    # 再次测试
    if python3 -c "from openarm.can import OpenArm" 2>/dev/null; then
        echo "  ✓ openarm_can Python 绑定构建成功"
    else
        echo "  ❌ 构建后仍然无法导入"
        echo ""
        echo "  请检查："
        echo "  1. 是否安装了必要的构建工具（meson, ninja）"
        echo "  2. 查看 build.sh 输出中的错误信息"
        exit 1
    fi
fi

# 测试 SDK 导入
echo ""
echo "[5/5] 测试 SDK 导入..."

cd "$WS_DIR/openarm_sdk"

# 添加 openarm_can 路径到 PYTHONPATH
export PYTHONPATH="$CAN_PYTHON_DIR:$PYTHONPATH"

if python3 -c "from openarm_sdk import OpenArmSDK" 2>/dev/null; then
    echo "  ✓ SDK 导入成功"
else
    echo "  ⚠️  SDK 导入失败"
    echo "  尝试安装 SDK: pip3 install -e ."
    read -p "  是否现在安装 SDK？(y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip3 install -e .
    else
        echo "  请手动安装 SDK"
        exit 1
    fi
fi

echo ""
echo "=========================================="
echo "✓ 所有检查通过！"
echo "=========================================="
echo ""
echo "下一步："
echo "  1. 设置 PYTHONPATH（如果尚未设置）"
echo "     export PYTHONPATH=\$HOME/ros2_ws/src/openarm_can/python:\$PYTHONPATH"
echo ""
echo "  2. 测试配置加载（无需硬件）"
echo "     python3 examples/config_example.py"


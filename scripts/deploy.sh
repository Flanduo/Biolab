#!/bin/bash
# Biolab 部署脚本
# 在开发服务器上运行，完成项目部署

set -e

BIOLAB_DIR="${1:-$HOME/Biolab}"

echo "=== Biolab 部署 ==="
echo "目标目录: $BIOLAB_DIR"

# 检查 git
if ! command -v git &> /dev/null; then
    echo "[ERROR] git 未安装"
    exit 1
fi

# 检查 conda
if ! command -v conda &> /dev/null; then
    echo "[WARN] conda 未安装，请先安装 Miniconda"
fi

# 检查 ROS2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "[WARN] ROS2 Humble 未安装，请参照 server_deployment_plan.md 安装"
fi

echo "=== 部署检查完成 ==="

#!/bin/bash
# Biolab 环境初始化脚本
# 在新机器上一键配置开发环境

set -e

echo "=== Biolab 环境初始化 ==="

# 激活 conda 环境
if [ -d "$HOME/Biolab/conda_envs/ros_env" ]; then
    source "$HOME/Biolab/conda_envs/ros_env/bin/activate"
    echo "[OK] conda 环境已激活"
else
    echo "[WARN] conda 环境未找到，请先部署 conda_envs/ros_env"
fi

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] ROS2 Humble 已加载"
else
    echo "[WARN] ROS2 Humble 未安装"
fi

# Source 工作空间
if [ -f "$HOME/Biolab/ros2_ws/install/setup.bash" ]; then
    source "$HOME/Biolab/ros2_ws/install/setup.bash"
    echo "[OK] ROS2 工作空间已加载"
fi

echo "=== 初始化完成 ==="

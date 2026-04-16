#!/bin/bash
# 打包 conda 环境用于迁移
# 在源机器 (10.0.0.2) 上运行

set -e

ENV_NAME="${1:-ros_env}"
OUTPUT="/tmp/ros_env.tar.gz"

echo "=== 打包 conda 环境: $ENV_NAME ==="

conda pack -n "$ENV_NAME" -o "$OUTPUT"
echo "[OK] 已输出到 $OUTPUT"
echo "传输到目标机器: scp $OUTPUT <user>@10.0.0.19:/tmp/"

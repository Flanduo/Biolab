#!/bin/bash
# Biolab 硬件调试停止脚本
# 停止所有 tmux 会话中的服务

SESSION="biolab_hardware"

echo "正在停止 Biolab 硬件服务..."

if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
    echo "✅ tmux 会话 '$SESSION' 已关闭"
else
    echo "会话 '$SESSION' 不存在"
fi

echo "完成"

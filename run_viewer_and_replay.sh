#!/bin/bash
# 同时启动多相机 Viewer 和 CSV Replay
# 用法: bash run_viewer_and_replay.sh

SESSION_ID="20260515_162747"

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VIEWER="$ROOT_DIR/video_record/multi_cam_viewer.py"
CSV="$ROOT_DIR/video_record/processed_dataset/${SESSION_ID}/aligned_data.csv"
REPLAY="$ROOT_DIR/openarm_demo/replay_csv_improved.py"

if [ ! -f "$CSV" ]; then
    echo "错误: 找不到 $CSV"
    exit 1
fi

cleanup() {
    echo ""
    echo "[LAUNCHER] 正在关闭所有进程..."
    kill $VIEWER_PID 2>/dev/null
    wait $VIEWER_PID 2>/dev/null
    echo "[LAUNCHER] 已清理。"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "============================================================"
echo "  Session: $SESSION_ID"
echo "============================================================"

# 启动 Viewer (后台)
python3 "$VIEWER" &
VIEWER_PID=$!
echo "[LAUNCHER] Viewer 已启动 (PID: $VIEWER_PID)"
echo ""

# 启动 CSV Replay (前台，--yes 跳过确认)
echo "[LAUNCHER] 启动 Replay..."
python3 "$REPLAY" "$CSV" --yes

# Replay 结束，Viewer 继续运行
echo ""
echo "============================================================"
echo "  Replay 已结束，Viewer 仍在运行"
echo "  按 Ctrl+C 关闭 Viewer"
echo "============================================================"
wait

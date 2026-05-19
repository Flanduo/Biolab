#!/usr/bin/env python3
"""
检测 aligned_data.csv 中机械臂静止的时间段
用法:
  python3 detect_idle_frames.py <csv_path>

原理:
  计算每帧机械臂关节的帧间位移量，找到位移最小的2个连续区间，
  作为"等待"帧段输出，可直接传给 clean_hand_cmd.py
"""
import csv
import sys
import numpy as np


def main():
    if len(sys.argv) != 2:
        print(f"用法: {sys.argv[0]} <csv_path>")
        sys.exit(1)

    path = sys.argv[1]

    with open(path, 'r') as f:
        rows = list(csv.reader(f))

    header = rows[0]
    n_frames = len(rows) - 1

    # 找出所有机械臂关节状态列 (不含 cmd 和 hand)
    arm_cols = [i for i, h in enumerate(header)
                if h.startswith('openarm_') and 'cmd' not in h]

    print(f"CSV: {path}")
    print(f"总帧数: {n_frames}")
    print(f"机械臂状态列数: {len(arm_cols)}")

    # 计算每帧的帧间位移 (L2 norm of joint delta)
    displacements = np.zeros(n_frames)
    prev = np.array([float(rows[1][c]) for c in arm_cols])
    for i in range(1, n_frames):
        curr = np.array([float(rows[i + 1][c]) for c in arm_cols])
        displacements[i] = np.linalg.norm(curr - prev)
        prev = curr

    # 帧位移直方图，帮助确定阈值
    nonzero = displacements[displacements > 0]
    if len(nonzero) == 0:
        print("所有帧位移为0，无法检测")
        return

    # 阈值: 用所有帧位移的中位数的一小部分
    threshold = np.median(displacements[1:]) * 0.15
    threshold = max(threshold, 0.005)  # 下限

    print(f"静止阈值: {threshold:.5f} rad")

    # 标记静止帧
    is_idle = displacements <= threshold
    # 第0帧没有前一帧，默认算静止
    is_idle[0] = True

    # 合并连续静止帧为区间，要求至少 min_len 帧才算有效
    min_len = 15  # 至少 ~0.3s (50Hz)
    segments = []
    start = None
    for i in range(n_frames):
        if is_idle[i]:
            if start is None:
                start = i
        else:
            if start is not None and (i - start) >= min_len:
                segments.append((start, i - 1))
            start = None
    if start is not None and (n_frames - start) >= min_len:
        segments.append((start, n_frames - 1))

    if len(segments) < 2:
        # 降低 min_len 重试
        min_len = 5
        segments = []
        start = None
        for i in range(n_frames):
            if is_idle[i]:
                if start is None:
                    start = i
            else:
                if start is not None and (i - start) >= min_len:
                    segments.append((start, i - 1))
                start = None
        if start is not None and (n_frames - start) >= min_len:
            segments.append((start, n_frames - 1))

    # 按区间长度排序，取最长的2个
    segments.sort(key=lambda s: s[1] - s[0], reverse=True)

    print(f"\n检测到 {len(segments)} 个静止段:")
    for i, (s, e) in enumerate(segments):
        dur = (e - s + 1) / 50.0
        print(f"  段{i+1}: frame {s}-{e} ({e-s+1} 帧, {dur:.1f}s)")

    def center_crop(s, e, ratio=0.5):
        """取区间中心 ratio 比例的子段"""
        length = e - s + 1
        sub = max(1, int(length * ratio))
        mid = (s + e) // 2
        return mid - sub // 2, mid + (sub - sub // 2) - 1

    if len(segments) >= 2:
        top2 = sorted(segments[:2], key=lambda s: s[0])
        s1, e1 = center_crop(*top2[0])
        s2, e2 = center_crop(*top2[1])
        print(f"\n=== 推荐参数 (取中心50%) ===")
        print(f"python3 scripts/clean_hand_cmd.py {path} {s1} {e1} {s2} {e2}")
    else:
        print("\n静止段不足2个，请手动检查。")
        print(f"建议用以下命令查看帧位移分布:")
        print(f"  python3 -c \"import numpy as np; d=np.loadtxt('{path}',delimiter=',',skiprows=1,usecols={arm_cols[0]}); print('done')\"")


if __name__ == '__main__':
    main()

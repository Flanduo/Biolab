#!/usr/bin/env python3
"""
数据清洗工具：修改 aligned_data.csv 中的灵巧手指令列
用法:
  python3 clean_hand_cmd.py <csv_path> <range1_start> <range1_end> <range2_start> <range2_end>

示例:
  python3 clean_hand_cmd.py ../processed_dataset/20260506_222144/aligned_data.csv 360 450 1000 1080

功能:
  1. 倒数第六列 (right_hand_joint1_cmd_action) 在指定帧段内置为 0
  2. 后四列 (right_hand_joint3~6_cmd_action) 全部帧置为 0
"""
import csv
import sys


def main():
    if len(sys.argv) != 6:
        print(f"用法: {sys.argv[0]} <csv_path> <r1_start> <r1_end> <r2_start> <r2_end>")
        sys.exit(1)

    path = sys.argv[1]
    r1s, r1e = int(sys.argv[2]), int(sys.argv[3])
    r2s, r2e = int(sys.argv[4]), int(sys.argv[5])

    with open(path, 'r') as f:
        rows = list(csv.reader(f))

    header = rows[0]
    ncols = len(header)
    sixth_last = ncols - 6

    print(f"CSV: {path}")
    print(f"总帧数: {len(rows) - 1}")
    print(f"倒数第六列: {header[sixth_last]}")
    print(f"后四列: {header[-4:]}")
    print(f"帧段1: {r1s}-{r1e}, 帧段2: {r2s}-{r2e}")

    # 倒数第六列: 两个帧段内置0
    for frame in list(range(r1s, r1e + 1)) + list(range(r2s, r2e + 1)):
        row_idx = frame + 1  # row 0 is header, frame 0 is row 1
        if 1 <= row_idx < len(rows):
            rows[row_idx][sixth_last] = '0'

    # 后四列: 全部帧置0
    for i in range(1, len(rows)):
        for ci in range(ncols - 4, ncols):
            rows[i][ci] = '0'

    with open(path, 'w', newline='') as f:
        csv.writer(f).writerows(rows)

    print(f"完成: 倒数第六列 ({header[sixth_last]}) 帧段置0, 后四列全置0")


if __name__ == '__main__':
    main()

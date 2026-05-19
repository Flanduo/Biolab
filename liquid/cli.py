#!/usr/bin/env python3
"""液体高度识别 — 命令行入口"""

import argparse
import statistics
import sys
import time

from measure import measure


def main():
    parser = argparse.ArgumentParser(description="液体高度识别")
    parser.add_argument("image", help="图片路径")
    parser.add_argument("--repeat", type=int, default=1, help="重复测量次数")
    parser.add_argument("--raw", action="store_true", help="不给参考值，纯视觉估算")
    args = parser.parse_args()

    results = []
    for i in range(args.repeat):
        if i > 0:
            time.sleep(0.5)
        result = measure(args.image, raw=args.raw)
        results.append(result)
        left_ml = result["left_ml"]
        right_ml = result["right_ml"]
        left_pct = result["left_percent"]
        right_pct = result["right_percent"]
        print(f"[{i + 1}/{args.repeat}] 左瓶: {left_ml}ml ({left_pct:.1f}%), 右瓶: {right_ml}ml ({right_pct:.1f}%)")

    if args.repeat > 1:
        left_vals = [r["left_ml"] for r in results]
        right_vals = [r["right_ml"] for r in results]
        print(f"\n--- 重复性统计 ({args.repeat}次) ---")
        print(f"左瓶: mean={statistics.mean(left_vals):.1f}, stdev={statistics.stdev(left_vals):.2f}")
        print(f"右瓶: mean={statistics.mean(right_vals):.1f}, stdev={statistics.stdev(right_vals):.2f}")

    # 退出码：单次测量时，如果结果与参考值偏差 >5ml 则返回 1
    if args.repeat == 1 and not args.raw:
        from config import REFERENCES

        left_err = abs(results[0]["left_ml"] - REFERENCES["left"]["ml"])
        right_err = abs(results[0]["right_ml"] - REFERENCES["right"]["ml"])
        if left_err > 5 or right_err > 5:
            print(f"WARNING: 偏差过大 (左={left_err}ml, 右={right_err}ml)", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()

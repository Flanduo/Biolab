#!/usr/bin/env python3
"""RealSense D435I 快速测试脚本 — 彩色/深度/IMU

用法:
  python3 test_d435i.py                     # 默认 640x480@30fps
  python3 test_d435i.py -r 1280x720        # 指定分辨率
  python3 test_d435i.py -r 848x480 -f 60   # 指定分辨率+帧率
  python3 test_d435i.py --list              # 列出支持的分辨率
"""

import argparse
import sys
import numpy as np
import pyrealsense2 as rs

D435I_SN = "045322075680"

# D435I 支持的分辨率
COLOR_MODES = {
    "1920x1080": [30, 15, 6],
    "1280x720":  [30, 15, 6],
    "960x540":   [60, 30, 15, 6],
    "848x480":   [60, 30, 15, 6],
    "640x480":   [60, 30, 15, 6],
    "640x360":   [60, 30, 15, 6],
    "424x240":   [60, 30, 15, 6],
    "320x240":   [60, 30, 6],
    "320x180":   [60, 30, 6],
}

DEPTH_MODES = {
    "1280x720": [30, 15, 6],
    "848x480":  [90, 60, 30, 15, 6],
    "848x100":  [300, 100],
    "640x480":  [90, 60, 30, 15, 6],
    "640x360":  [90, 60, 30, 15, 6],
    "480x270":  [90, 60, 30, 15, 6],
    "424x240":  [90, 60, 30, 15, 6],
    "256x144":  [300, 90],
}


def list_modes():
    print("D435I 支持的分辨率:\n")
    print(f"{'分辨率':>12}  {'彩色 FPS':<25}  {'深度 FPS'}")
    print("-" * 65)
    all_res = sorted(set(list(COLOR_MODES.keys()) + list(DEPTH_MODES.keys())),
                     key=lambda s: tuple(int(x) for x in s.split("x")))
    for res in all_res:
        c_fps = str(COLOR_MODES.get(res, "-"))
        d_fps = str(DEPTH_MODES.get(res, "-"))
        print(f"{res:>12}  {c_fps:<25}  {d_fps}")


def parse_args():
    p = argparse.ArgumentParser(description="D435I 测试脚本")
    p.add_argument("-r", "--resolution", default="640x480",
                   help="分辨率 WxH (默认 640x480)")
    p.add_argument("-f", "--fps", type=int, default=30,
                   help="帧率 (默认 30)")
    p.add_argument("--no-imu", action="store_true",
                   help="跳过 IMU 测试")
    p.add_argument("--list", action="store_true",
                   help="列出支持的分辨率并退出")
    return p.parse_args()


def test_d435i(args):
    w, h = map(int, args.resolution.split("x"))
    fps = args.fps

    pipe = rs.pipeline()

    # --- 1. 彩色 + 深度 ---
    cfg = rs.config()
    cfg.enable_device(D435I_SN)
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)

    print(f"[1] 启动 D435I (SN: {D435I_SN}) 彩色+深度流 {w}x{h}@{fps}fps ...")
    try:
        pipe.start(cfg)
    except RuntimeError as e:
        print(f"[!] 启动失败: {e}")
        print(f"    可能 {w}x{h}@{fps} 不被支持，用 --list 查看可用分辨率")
        return

    print("[1] 采集 5 帧验证 ...")
    for i in range(5):
        frames = pipe.wait_for_frames()
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color:
            c = np.asanyarray(color.get_data())
            print(f"  帧{i+1} color: {color.width}x{color.height}, dtype={c.dtype}")
        if depth:
            d = np.asanyarray(depth.get_data())
            valid = d[d > 0]
            dmin, dmax = (valid.min(), valid.max()) if len(valid) else (0, 0)
            print(f"  帧{i+1} depth: {depth.width}x{depth.height}, "
                  f"min={dmin}mm, max={dmax}mm")

    depth = frames.get_depth_frame()
    cx, cy = depth.width // 2, depth.height // 2
    dist = depth.get_distance(cx, cy)
    print(f"[1] 中心点 ({cx},{cy}) 距离: {dist:.3f} m")
    pipe.stop()
    print("[1] 彩色+深度流 OK\n")

    # --- 2. IMU ---
    if not args.no_imu:
        cfg2 = rs.config()
        cfg2.enable_device(D435I_SN)
        cfg2.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
        cfg2.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

        print("[2] 启动 IMU 流 (accel 63Hz, gyro 200Hz) ...")
        pipe.start(cfg2)

        print("[2] 采集 5 组 IMU 数据 ...")
        for i in range(5):
            frames = pipe.wait_for_frames()
            for f in frames:
                mf = f.as_motion_frame()
                data = mf.get_motion_data()
                stream_type = "accel" if mf.profile.stream_type() == rs.stream.accel else "gyro "
                print(f"  {stream_type}: x={data.x:+.4f}, y={data.y:+.4f}, z={data.z:+.4f}")

        pipe.stop()
        print("[2] IMU OK\n")

    # --- 3. 相机内参 (读取当前分辨率) ---
    cfg3 = rs.config()
    cfg3.enable_device(D435I_SN)
    cfg3.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    pipe.start(cfg3)
    frames = pipe.wait_for_frames()
    intrinsics = frames.get_color_frame().get_profile().as_video_stream_profile().get_intrinsics()
    print(f"[3] 彩色相机内参 ({w}x{h}):")
    print(f"    fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}")
    print(f"    ppx={intrinsics.ppx:.2f}, ppy={intrinsics.ppy:.2f}")
    print(f"    coeffs={list(intrinsics.coeffs)}")
    pipe.stop()
    print("[3] 内参读取 OK\n")

    print("=== D435I 全部测试通过 ===")


if __name__ == "__main__":
    args = parse_args()
    if args.list:
        list_modes()
        sys.exit(0)
    test_d435i(args)

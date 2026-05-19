#!/usr/bin/env python3
"""
ZED 相机客户端

封装对 ZEDProject/capture_server HTTP API 的调用，供全流程脚本使用。
"""

import os
import requests
from dataclasses import dataclass
from typing import Optional


@dataclass
class CaptureResult:
    """拍照结果"""
    success: bool
    color_file: str
    depth_file: str
    color_url: str
    depth_url: str
    resolution: str
    valid_depth_pixels: int
    total_pixels: int
    transferred: bool = False
    error: Optional[str] = None


class CameraClient:
    """ZED 相机服务客户端"""

    def __init__(self, host: str = "localhost", port: int = 5050):
        self.base_url = f"http://{host}:{port}"

    def _get(self, path: str, **kwargs) -> dict:
        r = requests.get(f"{self.base_url}{path}", timeout=10, **kwargs)
        r.raise_for_status()
        return r.json()

    def _post(self, path: str, **kwargs) -> dict:
        r = requests.post(f"{self.base_url}{path}", timeout=30, **kwargs)
        r.raise_for_status()
        return r.json()

    # ── 状态 ──

    def is_ready(self) -> bool:
        """检查相机服务是否就绪"""
        try:
            status = self._get("/status")
            return status.get("camera", {}).get("ready", False)
        except Exception:
            return False

    def get_status(self) -> dict:
        """获取相机完整状态"""
        return self._get("/status")

    def get_intrinsics(self) -> dict:
        """获取相机内参 (fx, fy, cx, cy 矩阵)"""
        return self._get("/intrinsics")

    # ── 拍照 ──

    def capture(self, object_name: str, transfer: bool = False) -> CaptureResult:
        """
        拍摄彩色图 + 深度图

        Args:
            object_name: 物体名称，用于文件命名
            transfer: 是否自动 SCP 传输到主控机 10.0.0.18
        """
        data = self._post("/capture", json={
            "object": object_name,
            "transfer": transfer,
        })
        urls = data.get("download_urls", {})
        return CaptureResult(
            success=True,
            color_file=data.get("color_file", ""),
            depth_file=data.get("depth_file", ""),
            color_url=urls.get("color", ""),
            depth_url=urls.get("depth", ""),
            resolution=data.get("resolution", ""),
            valid_depth_pixels=data.get("valid_depth_pixels", 0),
            total_pixels=data.get("total_pixels", 0),
            transferred=bool(data.get("transferred")),
            error=data.get("transfer_error"),
        )

    def list_captures(self) -> list:
        """列出所有已拍摄的文件"""
        return self._get("/captures").get("files", [])

    def download(self, filename: str, save_dir: str = ".") -> str:
        """
        下载拍摄的文件到本地

        Returns:
            保存的本地路径
        """
        r = requests.get(f"{self.base_url}/captures/{filename}", timeout=30)
        r.raise_for_status()
        path = os.path.join(save_dir, filename)
        with open(path, "wb") as f:
            f.write(r.content)
        return path

    # ── 录制 ──

    def start_recording(self) -> dict:
        """开始录制视频"""
        return self._post("/recording/start")

    def stop_recording(self) -> dict:
        """停止录制，返回文件名、时长、帧数"""
        return self._post("/recording/stop")

    def get_recording_status(self) -> dict:
        """查询录制状态"""
        return self._get("/recording/status")

    # ── 传输 ──

    def transfer(self, filenames: list) -> dict:
        """手动传输文件到主控机"""
        return self._post("/transfer", json={"files": filenames})


if __name__ == "__main__":
    cam = CameraClient()

    print("=== 相机状态 ===")
    print(f"就绪: {cam.is_ready()}")
    if cam.is_ready():
        print(f"状态: {cam.get_status()}")

        print("\n=== 内参 ===")
        print(cam.get_intrinsics())

        print("\n=== 拍照测试 ===")
        result = cam.capture("test_client")
        print(f"成功: {result.success}, 文件: {result.color_file}, {result.depth_file}")

        print("\n=== 录制测试 ===")
        cam.start_recording()
        import time
        time.sleep(2)
        info = cam.stop_recording()
        print(f"录制完成: {info.get('filename')}, {info.get('duration')}秒, {info.get('frame_count')}帧")

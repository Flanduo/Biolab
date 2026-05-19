"""液体高度识别 — 核心测量模块"""

import base64
import io
import json
import re

import requests
from PIL import Image

from config import (
    API_KEY,
    API_URL,
    IMAGE_MAX_SIZE,
    IMAGE_QUALITY,
    MODEL,
    PROMPT_RAW,
    PROMPT_WITH_REF,
    REFERENCES,
    TEMPERATURE,
    TOTAL_RANGE_ML,
)


def compress_image(image_path: str) -> str:
    """读取图片并压缩，返回 base64 编码的 JPEG。"""
    img = Image.open(image_path)
    if img.mode == "RGBA":
        img = img.convert("RGB")
    # 等比缩放
    w, h = img.size
    scale = min(IMAGE_MAX_SIZE / max(w, h), 1.0)
    if scale < 1.0:
        img = img.resize((int(w * scale), int(h * scale)), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=IMAGE_QUALITY)
    return base64.b64encode(buf.getvalue()).decode("utf-8")


def build_prompt(raw: bool = False) -> str:
    """构建 prompt。raw=True 时不含参考值。"""
    if raw:
        return PROMPT_RAW
    left = REFERENCES["left"]
    right = REFERENCES["right"]
    return PROMPT_WITH_REF.format(
        total=TOTAL_RANGE_ML,
        left_color=left["color"],
        left_ml=left["ml"],
        right_color=right["color"],
        right_ml=right["ml"],
    )


def call_api(img_b64: str, prompt: str) -> dict:
    """调用智谱视觉 API，返回解析后的 JSON dict。"""
    if not API_KEY:
        raise RuntimeError("ZHIPU_API_KEY environment variable is required")

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json",
    }
    payload = {
        "model": MODEL,
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"},
                    },
                    {"type": "text", "text": prompt},
                ],
            }
        ],
        "temperature": TEMPERATURE,
    }
    resp = requests.post(API_URL, headers=headers, json=payload, timeout=60)
    resp.raise_for_status()
    text = resp.json()["choices"][0]["message"]["content"]
    # 提取 JSON（模型可能在前后加文字）
    match = re.search(r"\{[^}]+\}", text)
    if not match:
        raise ValueError(f"无法从 API 响应中提取 JSON: {text}")
    return json.loads(match.group())


def measure(image_path: str, raw: bool = False) -> dict:
    """
    测量液体高度。

    Args:
        image_path: 图片文件路径
        raw: True 时不给参考值，纯视觉估算

    Returns:
        {"left_ml": ..., "right_ml": ..., "left_percent": ..., "right_percent": ...}
    """
    img_b64 = compress_image(image_path)
    prompt = build_prompt(raw=raw)
    return call_api(img_b64, prompt)

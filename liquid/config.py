"""液体高度识别 — 配置"""

import os

API_KEY = os.environ.get("ZHIPU_API_KEY", "")
API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
MODEL = "glm-4.6v-flash"
TEMPERATURE = 0.1

# 参考锚点（瓶子的已知液面）
REFERENCES = {
    "left": {"ml": 150, "color": "红色"},
    "right": {"ml": 200, "color": "黄色"},
}
TOTAL_RANGE_ML = 225  # 矩形段总量程

# 图片压缩参数
IMAGE_MAX_SIZE = 480   # 长边最大像素
IMAGE_QUALITY = 85     # JPEG 质量

# Prompt 模板
PROMPT_WITH_REF = (
    "这是一张正视相机拍摄的图片，图中有两个透明瓶，瓶身有矩形刻度段，总量程为{total}ml。\n"
    "已知参考值：左瓶（{left_color}液体）= {left_ml}ml，右瓶（{right_color}液体）= {right_ml}ml。\n\n"
    "请分别估算两个瓶子的液体体积（ml），只输出JSON，格式如下：\n"
    '{{"left_ml": 数字, "right_ml": 数字, "left_percent": 数字, "right_percent": 数字}}\n'
    "其中 percent 是液体占矩形段总高度的百分比(0-100)。不要输出其他内容。"
)

PROMPT_RAW = (
    "这是一张正视相机拍摄的图片，图中有两个透明瓶，瓶身有矩形刻度段，总量程为225ml。\n\n"
    "请分别估算两个瓶子的液体体积（ml），只输出JSON，格式如下：\n"
    '{{"left_ml": 数字, "right_ml": 数字, "left_percent": 数字, "right_percent": 数字}}\n'
    "其中 percent 是液体占矩形段总高度的百分比(0-100)。不要输出其他内容。"
)

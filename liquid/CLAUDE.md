# CLAUDE.md — 液体高度识别（视觉大模型方案）

## 项目目标

两个固定位置的透明瓶（矩形段量程 225ml），用相机正视拍照，调用智谱 GLM-4V-Flash 视觉 API 识别液面高度。

核心思路：用已知液面作为参照锚点（左瓶=150ml 红色, 右瓶=200ml 黄色），让视觉大模型直接读数。

## 文件结构

```
~/Biolab/liquid/
├── CLAUDE.md                    # 本文件，项目说明
├── config.py                    # API 配置、参考值、prompt 模板
├── measure.py                   # 核心测量：图片压缩 → base64 → API → JSON 解析
├── cli.py                       # 命令行入口
├── server.py                    # Flask API 服务（端口 5060）
└── realsense_d435i_color.png    # 测试图片（左=150ml, 右=200ml）
```

## 核心模块

### config.py
- 智谱 API 密钥、模型名 `glm-4v-flash`、temperature=0.1
- 参考锚点：左瓶 150ml 红色，右瓶 200ml 黄色，总量程 225ml
- 图片压缩：长边 ≤480px，JPEG quality=85
- 两个 prompt 模板：PROMPT_WITH_REF（含参考值）、PROMPT_RAW（纯视觉）

### measure.py
- `compress_image(path) → base64` — 读取图片，等比缩放，JPEG 压缩，base64 编码
- `build_prompt(raw=False) → str` — 构建 prompt
- `call_api(img_b64, prompt) → dict` — 调智谱 API，提取 JSON 响应
- `measure(image_path, raw=False) → dict` — 主入口，返回 `{"left_ml", "right_ml", "left_percent", "right_percent"}`

### cli.py
```bash
python3 cli.py <图片路径>                  # 单次测量
python3 cli.py <图片路径> --repeat 5       # 重复 5 次测重复性
python3 cli.py <图片路径> --raw            # 不给参考值，纯视觉估算
```

### server.py
```bash
python3 server.py   # 启动 Flask 服务，端口 5060

# POST /measure — 通过路径测量
curl -X POST http://localhost:5060/measure \
  -H 'Content-Type: application/json' \
  -d '{"image_path": "/home/elwg/Biolab/liquid/realsense_d435i_color.png"}'

# POST /measure_upload — 通过上传文件测量
curl -X POST http://localhost:5060/measure_upload \
  -F "image=@realsense_d435i_color.png"
```

## 当前状态

### 已验证
- 代码在 10.0.0.19 上正常运行
- API 调用成功，5 次重复 stdev=0（完美重复性）
- 依赖已满足：Python 3.10, Pillow 9.0.1, requests 2.25.1, Flask 3.1.3

### 已知问题 — 测量精度偏差
- 模型返回：左瓶≈120ml（参考 150ml，差 30ml），右瓶≈180ml（参考 200ml，差 20ml）
- 重复性完美但系统性偏低
- 可能原因：glm-4v-flash 模型版本更新、图片压缩丢刻度细节、prompt 措辞
- 调参方向：改 config.py 中的 prompt 模板、IMAGE_MAX_SIZE、IMAGE_QUALITY

### 未验证
- `cli.py --raw` 纯视觉估算模式
- `server.py` Flask API 服务端到端测试

## 待办
1. 调准测量精度（优化 prompt 或压缩参数）
2. 接入实际相机拍照流程（目前只有测试图片）
3. 验证 server.py 端到端

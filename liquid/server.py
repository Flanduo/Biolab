"""液体高度识别 — Flask API 服务"""

import os
import tempfile

from flask import Flask, jsonify, request
from werkzeug.utils import secure_filename

from measure import measure

app = Flask(__name__)


@app.route("/measure", methods=["POST"])
def measure_by_path():
    """通过图片路径测量。body: {"image_path": "/path/to/image.png"}"""
    data = request.get_json(force=True)
    image_path = data.get("image_path")
    if not image_path or not os.path.isfile(image_path):
        return jsonify({"error": "image_path 无效或文件不存在"}), 400
    result = measure(image_path)
    return jsonify(result)


@app.route("/measure_upload", methods=["POST"])
def measure_by_upload():
    """通过上传图片文件测量。multipart form, field name: image"""
    if "image" not in request.files:
        return jsonify({"error": "未找到 image 字段"}), 400
    f = request.files["image"]
    if not f.filename:
        return jsonify({"error": "文件名为空"}), 400
    # 保存到临时文件
    suffix = os.path.splitext(secure_filename(f.filename))[1]
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        f.save(tmp)
        tmp_path = tmp.name
    try:
        result = measure(tmp_path)
        return jsonify(result)
    finally:
        os.unlink(tmp_path)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5060, debug=True)

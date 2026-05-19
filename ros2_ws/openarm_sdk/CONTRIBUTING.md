# 为 OpenArm SDK 做贡献

感谢您对 OpenArm SDK 的贡献！

## 开发环境设置

1. 克隆仓库（或确保您在 ros2_ws 目录中）
2. 安装开发依赖：
   ```bash
   cd openarm_sdk
   pip install -e ".[dev]"
   ```
3. 构建 openarm_can Python 绑定（如果尚未构建）：
   ```bash
   cd ../src/openarm_can/python
   ./build.sh
   ```

## 代码风格

- 遵循 PEP 8 代码风格指南
- 尽可能使用类型提示
- 为所有公共函数和类编写文档字符串（使用中文注释）
- 使用 `black` 进行代码格式化
- 使用 `flake8` 进行代码检查

## 文档

- 添加新功能时更新文档字符串
- 如果添加重要功能，更新 README.md
- 在 `examples/` 目录中添加示例

## Pull Request 流程

1. 创建功能分支
2. 进行更改
3. 更新文档
4. 提交带有清晰描述的 pull request

## 有问题？

如有疑问或需要帮助，请提交 issue！


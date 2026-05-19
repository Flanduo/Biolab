# 更新日志

OpenArm SDK 的所有重要更改都将记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)，
本项目遵循 [语义化版本](https://semver.org/spec/v2.0.0.html)。

## [0.1.0] - 2025-01-XX

### 新增
- 初始 SDK 发布
- 用于机器人控制的基本 OpenArmSDK 类
- 配置管理（ArmConfig, ControlMode）
- 基本电机控制操作（使能、禁用、设置零位）
- 基本使用示例脚本
- 异常处理框架

## [0.2.0] - 2025-01-XX

### 新增
- ✅ YAML 配置加载器（ConfigLoader）
- ✅ 控制参数管理（ControlParameters）
- ✅ 控制循环框架（ControlLoop）
- ✅ 重力补偿控制实现（GravityCompensationControl）
- ✅ 动力学计算接口（DynamicsInterface, CallbackDynamics, SimpleDynamics）
- ✅ 简化动力学模型（用于快速原型）
- ✅ 配置加载示例（config_example.py）
- ✅ 双边控制框架示例（bilateral_control.py）

### 待办事项
- [ ] 完整实现重力补偿（需要动力学库支持）
- [ ] 双边控制支持
- [ ] 单边控制支持
- [ ] 遥操作功能
- [ ] 完整文档
- [ ] 单元测试


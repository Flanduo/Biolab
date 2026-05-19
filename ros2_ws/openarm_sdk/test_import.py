#!/usr/bin/env python3
"""
SDK 导入测试脚本

快速测试 SDK 是否安装正确，无需硬件连接。

使用方法：
    python3 test_import.py
"""

import sys

def test_imports():
    """测试所有导入"""
    print("=" * 60)
    print("OpenArm SDK 导入测试")
    print("=" * 60)
    
    errors = []
    
    # 测试 1: 基础模块
    print("\n[1/6] 测试基础模块导入...")
    try:
        from openarm_sdk import OpenArmSDK
        print("  ✓ OpenArmSDK")
    except Exception as e:
        print(f"  ❌ OpenArmSDK: {e}")
        errors.append("OpenArmSDK")
    
    # 测试 2: 配置相关
    print("\n[2/6] 测试配置模块导入...")
    try:
        from openarm_sdk import ArmConfig, ControlMode
        print("  ✓ ArmConfig, ControlMode")
    except Exception as e:
        print(f"  ❌ 配置模块: {e}")
        errors.append("配置模块")
    
    # 测试 3: 控制相关
    print("\n[3/6] 测试控制模块导入...")
    try:
        from openarm_sdk import ControlLoop, GravityCompensationControl
        print("  ✓ ControlLoop, GravityCompensationControl")
    except Exception as e:
        print(f"  ❌ 控制模块: {e}")
        errors.append("控制模块")
    
    # 测试 4: 配置加载器
    print("\n[4/6] 测试配置加载器导入...")
    try:
        from openarm_sdk import ConfigLoader, ControlParameters
        print("  ✓ ConfigLoader, ControlParameters")
    except Exception as e:
        print(f"  ❌ 配置加载器: {e}")
        errors.append("配置加载器")
    
    # 测试 5: 动力学接口
    print("\n[5/6] 测试动力学接口导入...")
    try:
        from openarm_sdk import DynamicsInterface, CallbackDynamics, SimpleDynamics
        print("  ✓ DynamicsInterface, CallbackDynamics, SimpleDynamics")
    except Exception as e:
        print(f"  ❌ 动力学接口: {e}")
        errors.append("动力学接口")
    
    # 测试 6: 异常类
    print("\n[6/6] 测试异常类导入...")
    try:
        from openarm_sdk import (
            OpenArmSDKError, MotorError, ConnectionError, 
            ConfigurationError, ControlError
        )
        print("  ✓ 所有异常类")
    except Exception as e:
        print(f"  ❌ 异常类: {e}")
        errors.append("异常类")
    
    # 测试依赖
    print("\n[额外] 测试依赖包...")
    try:
        import yaml
        print("  ✓ PyYAML")
    except ImportError:
        print("  ❌ PyYAML（需要安装: pip install pyyaml）")
        errors.append("PyYAML")
    
    try:
        import numpy
        print("  ✓ NumPy")
    except ImportError:
        print("  ⚠️  NumPy（可选，推荐安装）")
    
    # 测试 openarm_can（如果可用）
    print("\n[额外] 测试 openarm_can Python 绑定...")
    try:
        import sys
        from pathlib import Path
        
        # 尝试添加路径
        can_path = Path(__file__).parent.parent / "src" / "openarm_can" / "python"
        if can_path.exists():
            sys.path.insert(0, str(can_path))
        
        from openarm.can import OpenArm
        print("  ✓ openarm_can Python 绑定可用")
    except ImportError as e:
        print(f"  ⚠️  openarm_can Python 绑定不可用: {e}")
        print("  （需要构建: cd src/openarm_can/python && ./build.sh）")
    
    # 总结
    print("\n" + "=" * 60)
    if errors:
        print("❌ 测试失败！")
        print(f"失败的模块: {', '.join(errors)}")
        print("\n解决方案：")
        print("  1. 确保已安装 SDK: pip install -e .")
        print("  2. 安装依赖: pip install pyyaml numpy")
        print("  3. 检查 Python 版本（需要 3.8+）")
        return False
    else:
        print("✓ 所有导入测试通过！")
        print("\n下一步：")
        print("  1. 运行 python3 examples/config_example.py（测试配置加载）")
        print("  2. 连接硬件后运行 python3 examples/basic_control.py")
        return True


if __name__ == "__main__":
    success = test_imports()
    sys.exit(0 if success else 1)


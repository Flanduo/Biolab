#!/usr/bin/env python3
"""调试导入问题的测试脚本"""

import sys
import traceback

print("=" * 60)
print("测试 openarm_sdk 导入")
print("=" * 60)

# 测试 1: 导入包
print("\n1. 测试导入 openarm_sdk 包...")
try:
    import openarm_sdk
    print("✓ openarm_sdk 包导入成功")
    print(f"  包位置: {openarm_sdk.__file__}")
    print(f"  包内容: {dir(openarm_sdk)}")
except Exception as e:
    print(f"❌ 导入失败: {e}")
    traceback.print_exc()
    sys.exit(1)

# 测试 2: 导入 core 模块
print("\n2. 测试导入 core 模块...")
try:
    from openarm_sdk import core
    print("✓ core 模块导入成功")
    print(f"  core 模块内容: {[x for x in dir(core) if not x.startswith('_')]}")
except Exception as e:
    print(f"❌ core 模块导入失败: {e}")
    traceback.print_exc()
    sys.exit(1)

# 测试 3: 导入 OpenArmSDK
print("\n3. 测试导入 OpenArmSDK 类...")
try:
    from openarm_sdk.core import OpenArmSDK
    print("✓ OpenArmSDK 类导入成功")
except Exception as e:
    print(f"❌ OpenArmSDK 类导入失败: {e}")
    traceback.print_exc()
    sys.exit(1)

# 测试 4: 从包根导入 OpenArmSDK
print("\n4. 测试从包根导入 OpenArmSDK...")
try:
    from openarm_sdk import OpenArmSDK
    print("✓ 从包根导入 OpenArmSDK 成功")
    print(f"  OpenArmSDK 类: {OpenArmSDK}")
except Exception as e:
    print(f"❌ 从包根导入失败: {e}")
    traceback.print_exc()
    print("\n尝试直接检查包内容...")
    import openarm_sdk
    print(f"  openarm_sdk.__all__: {openarm_sdk.__all__}")
    print(f"  'OpenArmSDK' in dir(openarm_sdk): {'OpenArmSDK' in dir(openarm_sdk)}")
    if 'OpenArmSDK' in dir(openarm_sdk):
        print(f"  getattr(openarm_sdk, 'OpenArmSDK'): {getattr(openarm_sdk, 'OpenArmSDK', None)}")
    sys.exit(1)

print("\n" + "=" * 60)
print("所有导入测试通过！")
print("=" * 60)


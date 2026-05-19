"""
控制模块

本模块提供了高级控制功能，包括重力补偿、双边控制等。
目前提供基础框架，具体实现需要根据需求逐步完善。
"""

import time
import threading
from typing import Optional, Callable, List
import numpy as np
from .exceptions import ControlError, ConfigurationError
from .dynamics import DynamicsInterface, SimpleDynamics


class ControlLoop:
    """
    控制循环基类
    
    提供可继承的控制循环框架，支持定时执行控制逻辑。
    """
    
    def __init__(self, frequency: float = 100.0):
        """
        初始化控制循环
        
        参数:
            frequency: 控制频率（Hz），默认 100Hz
        """
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.running = False
        self._thread: Optional[threading.Thread] = None
    
    def step(self):
        """
        控制步骤
        
        子类需要实现此方法，定义每个控制周期的操作。
        """
        raise NotImplementedError("子类必须实现 step() 方法")
    
    def _run_loop(self):
        """运行控制循环（内部方法）"""
        while self.running:
            start_time = time.time()
            try:
                self.step()
            except Exception as e:
                # 记录错误但不中断循环
                print(f"控制步骤执行错误: {e}")
            
            # 保持固定频率
            elapsed = time.time() - start_time
            sleep_time = max(0, self.period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def start(self):
        """启动控制循环"""
        if self.running:
            return
        
        self.running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """停止控制循环"""
        self.running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None


class GravityCompensationControl(ControlLoop):
    """
    重力补偿控制
    
    实现重力补偿控制模式，允许用户轻松拖动机械臂。
    支持通过动力学接口或回调函数注入动力学计算。
    """
    
    def __init__(
        self, 
        openarm_sdk, 
        dynamics: Optional[DynamicsInterface] = None,
        frequency: float = 100.0,
    ):
        """
        初始化重力补偿控制
        
        参数:
            openarm_sdk: OpenArmSDK 实例
            dynamics: 动力学计算接口（可选，默认使用简化模型）
            frequency: 控制频率（Hz），默认 100Hz
        """
        super().__init__(frequency)
        self.sdk = openarm_sdk
        self._openarm = openarm_sdk._openarm
        
        # 动力学计算接口
        if dynamics is None:
            # 获取电机数量
            arm = self._openarm.get_arm()
            num_motors = len(arm.get_motors())
            self.dynamics = SimpleDynamics(num_joints=num_motors)
        else:
            self.dynamics = dynamics
        
        # 预分配数组
        arm = self._openarm.get_arm()
        num_motors = len(arm.get_motors())
        self._joint_positions = [0.0] * num_motors
        self._grav_torques = [0.0] * num_motors
        
        # 统计信息
        self._frame_count = 0
        self._start_time = None
    
    def step(self):
        """执行重力补偿控制步骤"""
        try:
            # 刷新电机状态
            self._openarm.refresh_all()
            self._openarm.recv_all(timeout_us=100)
            
            # 获取电机位置
            arm = self._openarm.get_arm()
            motors = arm.get_motors()
            
            # 读取电机位置
            for i, motor in enumerate(motors):
                if i < len(self._joint_positions):
                    self._joint_positions[i] = motor.get_position()
            
            # 计算重力补偿扭矩
            self._grav_torques = self.dynamics.get_gravity(self._joint_positions)
            
            # 准备 MIT 控制参数
            # MITParam 结构: MITParam(kp, kd, q, dq, tau)
            # 对于重力补偿，我们只需要前馈扭矩（tau=重力扭矩），其他为0
            try:
                from openarm.can import MITParam
            except ImportError:
                # 如果无法导入，使用占位符
                raise ControlError(
                    "无法导入 MITParam。请确保 openarm_can Python 绑定已构建并安装。"
                )
            
            cmds = []
            for i, torque in enumerate(self._grav_torques):
                if i < len(motors):
                    # 创建 MIT 控制参数：kp=0, kd=0, q=0, dq=0, tau=gravity_torque
                    cmd = MITParam(kp=0.0, kd=0.0, q=0.0, dq=0.0, tau=torque)
                    cmds.append(cmd)
            
            # 发送控制命令
            if cmds:
                arm.mit_control_all(cmds)
            
            # 更新统计信息
            self._frame_count += 1
            if self._start_time is None:
                self._start_time = time.time()
            
            # 每秒显示一次频率统计
            elapsed = time.time() - self._start_time
            if elapsed >= 1.0 and self._frame_count > 0:
                actual_hz = self._frame_count / elapsed
                print(f"=== 控制循环频率: {actual_hz:.1f} Hz (目标: {self.frequency} Hz) ===")
                self._frame_count = 0
                self._start_time = time.time()
            
        except Exception as e:
            raise ControlError(f"重力补偿控制步骤失败: {e}") from e


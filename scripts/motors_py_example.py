#!/usr/bin/env python3
import math
import sys
import time
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[1]
_motors_site = _repo_root / "install" / "motors" / "lib" / "python3.10" / "site-packages"
if _motors_site.exists() and str(_motors_site) not in sys.path:
    sys.path.insert(0, str(_motors_site))

import motors_py

def example_can_motor():
    """使用CAN总线连接的电机示例"""
    print("=== CAN电机示例 ===")
    motors = []
    try:
        for i in range(1, 2):
            motors.append(motors_py.MotorDriver.create_motor(
            motor_id=i,
            interface_type="can",
            interface="can0",
            motor_type="DM",
            motor_model=0,
            master_id_offset=16,
        ))
    except Exception as e:
        print(f"创建电机失败: {e}")
        return
    
    try:
        print("使能电机...")
        for motor in motors:
            motor.init_motor()
        
        print("\n=== MIT模式控制示例 ===")
        motors[0].set_motor_control_mode(motors_py.MotorControlMode.MIT)

        # 记录初始位置（使用相对位移，避免动作逐次变小）
        initial_pos = motors[0].get_motor_pos()

        amplitude = 1.5  # 正弦位移幅值（rad）
        frequency = 0.2  # 正弦频率（Hz）
        kp = 5.0
        kd = 1.0
        torque = 0.0
        dt = 0.01

        print("开始正弦正反转动，按 Ctrl+C 退出...\n")
        start_time = time.time()
        while True:
            t = time.time() - start_time
            target_pos = initial_pos + amplitude * math.sin(2.0 * math.pi * frequency * t)
            target_vel = 0.0
            motors[0].motor_mit_cmd(target_pos, target_vel, kp, kd, torque)
            time.sleep(dt)
            
        # 读取电机状态
        pos = motors[0].get_motor_pos()
        vel = motors[0].get_motor_spd()
        current = motors[0].get_motor_current()
        temp = motors[0].get_motor_temperature()
        error_id = motors[0].get_error_id()
        
        print(f"位置: {pos:.4f} rad, 速度: {vel:.4f} rad/s, "
              f"电流: {current:.4f} A, 温度: {temp:.2f}°C, 错误码: {error_id}")
        time.sleep(1)
    except Exception as e:
        print(f"电机控制过程中出错: {e}")
    finally:
        for motor in motors:
            print("失能电机...")
            motor.deinit_motor()


if __name__ == "__main__":
    example_can_motor()

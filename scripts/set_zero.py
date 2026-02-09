#!/usr/bin/env python3
import os
import sys
import yaml
import motors_py
import time


def load_config(config_path: str) -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config


def create_motors(config: dict) -> list:
    motors = []
    motor_ids = config['motor_id']
    motor_interface_type = config['motor_interface_type']
    motor_interfaces = config['motor_interface']
    motor_num = config['motor_num']
    motor_type = config['motor_type']
    motor_models = config['motor_model']
    master_id_offset = config['master_id_offset']
    motor_zero_offsets = config['motor_zero_offset']
    
    motor_idx = 0
    for interface_idx, num in enumerate(motor_num):
        interface = motor_interfaces[interface_idx]
        for _ in range(num):
            if motor_idx >= len(motor_ids):
                break
            motor_id = motor_ids[motor_idx]
            motor_model = motor_models[motor_idx] if motor_idx < len(motor_models) else 0
            motor_zero_offset = motor_zero_offsets[motor_idx] if motor_idx < len(motor_zero_offsets) else 0.0
            
            motor = motors_py.MotorDriver.create_motor(
                motor_id=motor_id,
                interface_type=motor_interface_type,
                interface=interface,
                motor_type=motor_type,
                motor_model=motor_model,
                master_id_offset=master_id_offset,
                motor_zero_offset=motor_zero_offset,
            )
            motors.append({
                'motor': motor,
                'motor_id': motor_id,
                'interface': interface,
                'index': motor_idx
            })
            motor_idx += 1
    
    return motors


def set_damping_mode(motor):
    motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
    motor.motor_mit_cmd(0.0, 0.0, 0.0, 2.0, 0.0)


def calibrate_motor(motor_info: dict):
    motor = motor_info['motor']
    motor_id = motor_info['motor_id']
    interface = motor_info['interface']
    
    print(f"\n{'='*50}")
    print(f"正在标定电机 ID: {motor_id} (接口: {interface})")
    print(f"{'='*50}")
    
    print("使能电机...")
    motor.init_motor()
    time.sleep(0.3)
    
    print("设置纯阻尼控制模式 (MIT: 0, 0, 0, 2, 0)...")
    set_damping_mode(motor)
    time.sleep(0.1)
    
    print("\n>>> 请手动将电机摆到零位 <<<")
    print("提示: 电机现在处于阻尼模式，可以自由转动")
    
    try:
        while True:
            motor.motor_mit_cmd(0.0, 0.0, 0.0, 2.0, 0.0)
            
            pos = motor.get_motor_pos()
            print(f"\r当前位置: {pos:+.6f} rad | 按 Enter 确认并标零...", end='', flush=True)
            
            import select
            if select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.readline()
                break
            
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n用户中断标定")
        motor.deinit_motor()
        raise
    
    motor.set_motor_zero()
    print(f"\n\n电机 {motor_id} 已标零!")
    
    print("失能电机...")
    motor.deinit_motor()
    time.sleep(0.2)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config', 'set_zero.yaml')
    
    print("="*60)
    print("           电机零点标定工具")
    print("="*60)
    print(f"\n配置文件: {config_path}\n")
    
    try:
        config = load_config(config_path)
    except Exception as e:
        print(f"加载配置文件失败: {e}")
        return 1
    
    print("配置信息:")
    print(f"  - 电机ID列表: {config['motor_id']}")
    print(f"  - 电机类型: {config['motor_type']}")
    print(f"  - 接口类型: {config['motor_interface_type']}")
    print(f"  - 接口: {config['motor_interface']}")
    print(f"  - 电机型号: {config['motor_model']}")
    print("\n" + "-"*60)
    input("按 Enter 开始标定流程...")
    print("-"*60)
    
    try:
        motors = create_motors(config)
        print(f"\n成功创建 {len(motors)} 个电机对象")
    except Exception as e:
        print(f"创建电机失败: {e}")
        return 1
    
    try:
        for motor_info in motors:
            calibrate_motor(motor_info)
            print(f"电机 {motor_info['motor_id']} 标定完成!")
    except KeyboardInterrupt:
        print("\n\n标定被用户中断")
        return 1
    except Exception as e:
        print(f"\n标定过程出错: {e}")
        return 1
    
    print("\n" + "="*60)
    print("         所有电机标零完成!")
    print("="*60)
    
    print("\n标定流程结束")
    return 0


if __name__ == '__main__':
    sys.exit(main())

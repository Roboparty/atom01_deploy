#!/usr/bin/env python3
import sys
import time
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[1]
_motors_site = _repo_root / "install" / "motors" / "lib" / "python3.10" / "site-packages"
if _motors_site.exists() and str(_motors_site) not in sys.path:
    sys.path.insert(0, str(_motors_site))

import motors_py


def enable_only_can_motors():
    """只使能电机，不执行运动指令，并输出未连接/不可用电机"""
    print("=== CAN电机使能检测 ===")
    motors = []
    missing_motors = []
    motor_model_map = [
        1, 1, 1, 1, 0, 0,
        1, 1, 1, 1, 0, 0, 1,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
    ]
    can_groups = {
        "can0": [1, 2, 3, 4, 5, 6],
        "can1": [7, 8, 9, 10, 11, 12, 13],
        "can2": [14, 15, 16, 17, 18],
        "can3": [19, 20, 21, 22, 23],
    }

    for interface, ids in can_groups.items():
        for motor_id in ids:
            motor_model = motor_model_map[motor_id - 1]
            try:
                motor = motors_py.MotorDriver.create_motor(
                    motor_id=motor_id,
                    interface_type="can",
                    interface=interface,
                    motor_type="DM",
                    motor_model=motor_model,
                    master_id_offset=16,
                )
                motors.append({"motor": motor, "id": motor_id, "interface": interface})
            except Exception as e:
                print(f"创建电机失败: id={motor_id}, interface={interface}, err={e}")
                missing_motors.append((motor_id, interface, f"create: {e}"))
                continue

    try:
        print("使能电机...")
        active_motors = []
        for item in motors:
            motor = item["motor"]
            try:
                motor.init_motor()
                active_motors.append(item)
            except Exception as e:
                motor_id = item["id"]
                interface = item["interface"]
                missing_motors.append((motor_id, interface, f"init: {e}"))
                print(f"使能电机失败: id={motor_id}, err={e}")

        # 探测电机是否在线（未响应的标记为缺失）
        verified_motors = []
        for item in active_motors:
            motor = item["motor"]
            motor_id = item["id"]
            interface = item["interface"]
            online = False
            for _ in range(3):
                motor.refresh_motor_status()
                time.sleep(0.05)
                if motor.get_response_count() == 0:
                    online = True
                    break
            if online:
                verified_motors.append(item)
            else:
                missing_motors.append((motor_id, interface, "no response"))
                print(f"电机无响应: id={motor_id}, interface={interface}")

        if not verified_motors:
            print("无可用电机。")
        else:
            print(f"可用电机数量: {len(verified_motors)}")

        # 不执行任何运动指令，仅保持使能短暂时间
        time.sleep(0.5)
    except Exception as e:
        print(f"电机使能过程中出错: {e}")
    finally:
        if missing_motors:
            print("\n缺少/不可用电机列表:")
            for motor_id, interface, reason in missing_motors:
                interface_text = interface if interface else "unknown"
                print(f"- id={motor_id}, interface={interface_text}, reason={reason}")

        for item in motors:
            print("失能电机...")
            item["motor"].deinit_motor()


if __name__ == "__main__":
    enable_only_can_motors()

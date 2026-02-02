#!/usr/bin/env python3
import imu_py
import time
import sys

def example_serial_imu():
    """使用串口连接的IMU示例"""
    print("=== 串口IMU示例 ===")
    try:
        imu = imu_py.IMUDriver.create_imu(
            imu_id=8,
            interface_type="serial",
            interface="/dev/ttyUSB0",
            imu_type="HIPNUC",
            baudrate=921600
        )
    except Exception as e:
        print(f"创建IMU失败: {e}")
        return
    
    print(f"IMU ID: {imu.get_imu_id()}")
    print("\n按 Ctrl+C 退出。")

    # 使用 ANSI 控制台刷新，减少屏幕闪烁
    sys.stdout.write("\x1b[?25l")  # 隐藏光标
    sys.stdout.write("\x1b[2J\x1b[H")  # 清屏并移动到左上角
    sys.stdout.flush()

    try:
        for i in range(1000):
            quat = imu.get_quat()
            ang_vel = imu.get_ang_vel()
            lin_acc = imu.get_lin_acc()
            temp = imu.get_temperature()

            output = (
                "IMU 实时数据\n"
                f"四元数(无量纲): w={quat[0]:.4f}, x={quat[1]:.4f}, y={quat[2]:.4f}, z={quat[3]:.4f}\n"
                f"角速度(rad/s): x={ang_vel[0]:.4f}, y={ang_vel[1]:.4f}, z={ang_vel[2]:.4f}\n"
                f"线加速度(m/s^2): x={lin_acc[0]:.4f}, y={lin_acc[1]:.4f}, z={lin_acc[2]:.4f}\n"
                f"温度(°C): {temp:.2f}\n"
                f"采样计数: {i + 1}\n"
            )

            sys.stdout.write("\x1b[H")  # 回到左上角覆盖输出
            sys.stdout.write(output)
            sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\x1b[?25h")  # 显示光标
        sys.stdout.flush()

if __name__ == "__main__":
    example_serial_imu()
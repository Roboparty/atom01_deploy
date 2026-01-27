# ATOM01 ROS2 Deploy

[![ROS2](https://img.shields.io/badge/ROS2-Humble-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
[![Linux platform](https://img.shields.io/badge/platform-linux--x86_64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--aarch64-orange.svg)](https://releases.ubuntu.com/22.04/)

[English](README.md) | [中文](README_CN.md)

## 概述

本仓库提供了使用ROS2作为中间件的部署框架，并具有模块化架构，便于无缝定制和扩展。

**维护者**: 刘志浩
**联系方式**: <ZhihaoLiu_hit@163.com>

**主要特性:**

- `易于上手` 提供全部细节代码，便于学习并允许修改代码。
- `隔离性` 不同功能由不同包实现，支持加入自定义功能包。
- `长期支持` 本仓库将随着训练仓库代码的更新而更新，并将长期支持。

## 环境配置

部署代码在香橙派5plus上运行，系统为ubuntu22.04，内核版本为5.10，我们在香橙派5plus上进行部署的环境配置。

首先安装ROS2 humble，参考[ROS官方](https://docs.ros.org/en/humble/Installation.html)进行安装。

部署还依赖ccache fmt spdlog eigen3等库，在上位机中执行指令进行安装：

```bash
sudo apt update && sudo apt install -y ccache libfmt-dev libspdlog-dev libeigen3-dev
```

然后安装香橙派5plus的5.10实时内核

```bash
git clone https://github.com/Roboparty/atom01_deploy.git
cd atom01_deploy/assets
sudo apt install ./*.deb
cd ..
```

接下来为用户授予实时优先级设置权限：

```bash
sudo nano /etc/security/limits.conf
```

在文件末尾添加以下两行，将 orangepi 替换为你的实际用户名：

```bash
# Allow user 'orangepi' to set real-time priorities
orangepi   -   rtprio   98
orangepi   -   memlock  unlimited
```

保存退出后重启香橙派使设置生效。

## 硬件链接

电机驱动中can0对应左腿，can1对应右腿加腰，can2对应左手，can3对应右手，默认按照usb转can插入上位机顺序编号，先插入的是can0。建议将USB转CAN插在上位机的3.0接口上，如果使用USB扩展坞也请使用3.0接口的USB扩展坞并插在3.0接口上，IMU和手柄插在USB2.0接口即可。

编写udev规则用来将USB口与usb转can绑定，即可不需要再管设备插入上位机顺序，示例在99-auto-up-devs.rules中，需要修改的是KERNELS项，将其修改为该usb转can想要对应绑定的USB接口的KERNELS属性项。在上位机输入指令监视USB事件：

```bash
sudo udevadm monitor
```

在USB上插入设备时就会显示该USB接口的KERNELS属性项，如/devices/pci0000:00/0000:00:14.0/usb3/3-8，我们在匹配KERNELS属性项时使用3-8即可。如果想要绑定在该USB接口上的扩展坞上的USB口则会有3-8.x出现，此时使用3-8.x进行匹配扩展坞上的USB口。

编写完成后在上位机中执行：

```bash
sudo cp 99-auto-up-devs.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

之后拔下所有can设备再插入指定USB接口即可生效。

该udev规则还包括IMU串口配置，也需要修改KERNELS项，方法不再赘述。如果规则正常生效，can应该全部自动配置完毕并使能，可以在上位机中输入ip a指令查看结果。

如果不配置udev规则，则需要按照上文顺序插入usb转can，并手动配置can和IMU串口：

```bash
sudo ip link set canX up type can bitrate 1000000
sudo ip link set canX txqueuelen 1000
# canX 为 can0 can1 can2 can3，需要为每个can都输入一遍上面两个指令
sudo chmod 666 /dev/ttyUSB0
```

## 软件使用

### 启动机器人

```bash
./tools/start_robot.sh
```

### 手柄控制

- **A 键**: 初始化/去初始化电机
- **X 键**: 重置电机
- **B 键**: 开始/暂停推理
- **Y 键**: 切换 手柄控制 / cmd_vel指令控制

### 服务接口

可以通过命令行调用ROS2服务来控制机器人：

- **初始化电机**:
  ```bash
  ros2 service call /init_motors std_srvs/srv/Trigger
  ```

- **去初始化电机**:
  ```bash
  ros2 service call /deinit_motors std_srvs/srv/Trigger
  ```

- **开始推理**:
  ```bash
  ros2 service call /start_inference std_srvs/srv/Trigger
  ```

- **停止推理**:
  ```bash
  ros2 service call /stop_inference std_srvs/srv/Trigger
  ```

- **清除错误**:
  ```bash
  ros2 service call /clear_errors std_srvs/srv/Trigger
  ```

- **设置零点**:
  ```bash
  ros2 service call /set_zeros std_srvs/srv/Trigger
  ```

- **重置关节**:
  ```bash
  ros2 service call /reset_joints std_srvs/srv/Trigger
  ```

- **刷新关节状态**:
  ```bash
  ros2 service call /refresh_joints std_srvs/srv/Trigger
  ```

## Python SDK

本仓库提供了 Python SDK，方便用户使用 Python 脚本控制硬件。在使用前请确保已经 source 了 ROS2 环境和本工作空间的 install/setup.bash。

### 1. IMU SDK (`imu_py`)

#### 静态方法
- `create_imu(imu_id: int, interface_type: str, interface: str, imu_type: str, baudrate: int = 0) -> IMUDriver`: 创建 IMU 驱动实例。

#### 成员方法
- `get_imu_id() -> int`: 获取 IMU ID。
- `get_ang_vel() -> List[float]`: 获取角速度 [x, y, z]。
- `get_quat() -> List[float]`: 获取四元数 [w, x, y, z]。
- `get_lin_acc() -> List[float]`: 获取线加速度 [x, y, z]。
- `get_temperature() -> float`: 获取温度。

#### 使用示例
```python
import imu_py
imu = imu_py.IMUDriver.create_imu(8, "serial", "/dev/ttyUSB0", "HIPNUC", 921600)
quat = imu.get_quat()
```

### 2. 电机 SDK (`motors_py`)

提供了 `MotorControlMode` 枚举：`NONE`, `MIT`, `POS`, `SPD`。

#### 静态方法
- `create_motor(motor_id: int, interface_type: str, interface: str, motor_type: str, motor_model: int, master_id_offset: int = 0) -> MotorDriver`: 创建电机驱动实例。

#### 成员方法
- `init_motor()`: 初始化电机。
- `deinit_motor()`: 去初始化电机。
- `set_motor_control_mode(mode: MotorControlMode)`: 设置控制模式。
- `motor_mit_cmd(pos: float, vel: float, kp: float, kd: float, torque: float)`: MIT 模式控制指令。
- `motor_pos_cmd(pos: float, spd: float, ignore_limit: bool = False)`: 位置模式控制指令。
- `motor_spd_cmd(spd: float)`: 速度模式控制指令。
- `lock_motor() / unlock_motor()`: 锁定/解锁电机。
- `set_motor_zero()`: 设置当前位置为零点。
- `clear_motor_error()`: 清除错误。
- `get_motor_pos() -> float`: 获取位置 (rad)。
- `get_motor_spd() -> float`: 获取速度 (rad/s)。
- `get_motor_current() -> float`: 获取电流 (A)。
- `get_motor_temperature() -> float`: 获取温度 (°C)。
- `get_error_id() -> int`: 获取错误码。
- `write_motor_flash()`: 将当前参数写入 Flash。
- `reset_motor_id(new_id: int)`: 重置电机 ID。

#### 使用示例
```python
import motors_py
motor = motors_py.MotorDriver.create_motor(1, "can", "can0", "DM", 0, 16)
motor.init_motor()
motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
motor.motor_mit_cmd(0.0, 0.0, 5.0, 1.0, 0.0)
```

### 3. 机器人 SDK (`robot_py`)

`RobotInterface` 类用于统一控制整个机器人，读取配置文件自动加载电机和 IMU。

#### 构造函数
- `RobotInterface(config_file: str)`: 根据配置文件路径创建实例。

#### 成员方法
- `init_motors()`: 初始化所有电机。
- `deinit_motors()`: 去初始化所有电机。
- `reset_joints(joint_default_angle: List[float])`: 将所有关节重置到默认角度。
- `apply_action(action: List[float])`: 应用控制动作 (关节目标位置/力矩等，取决于内部实现)。
- `refresh_joints()`: 刷新所有关节状态。
- `set_zeros()`: 将当前所有关节位置设为零点。
- `clear_errors()`: 清除所有电机错误。
- `get_joint_q() -> List[float]`: 获取所有关节位置。
- `get_joint_vel() -> List[float]`: 获取所有关节速度。
- `get_joint_tau() -> List[float]`: 获取所有关节力矩。
- `get_quat() -> List[float]`: 获取 IMU 四元数 [w, x, y, z]。
- `get_ang_vel() -> List[float]`: 获取 IMU 角速度。

#### 属性
- `is_init`: (只读) 机器人是否已初始化。

#### 使用示例
```python
import robot_py
robot = robot_py.RobotInterface("config/robot.yaml")
robot.init_motors()
robot.apply_action([0.0] * 23)
```

**提示**: 详细 Python 脚本示例请参考 `scripts/` 目录。

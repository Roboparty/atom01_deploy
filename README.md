# ATOM01 ROS2 Deploy

[![ROS2](https://img.shields.io/badge/ROS2-Humble-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
[![Linux platform](https://img.shields.io/badge/platform-linux--x86_64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--aarch64-orange.svg)](https://releases.ubuntu.com/22.04/)

[English](README.md) | [中文](README_CN.md)

## Overview

This repository provides a deployment framework using ROS2 as middleware with a modular architecture for seamless customization and extension.

**Maintainer**: Zhihao Liu
**Contact**: <ZhihaoLiu_hit@163.com>

**Key Features:**

- `Easy to Use` Provides complete detailed code for learning and code modification.
- `Isolation` Different functions are implemented by different packages, supporting custom function packages.
- `Long-term Support` This repository will be updated with the training repository code and will be supported long-term.

## Environment Setup

The deployment code runs on Orange Pi 5 Plus with Ubuntu 22.04 system and kernel version 5.10. We perform the environment configuration for deployment on Orange Pi 5 Plus.

First install ROS2 humble, refer to [ROS official](https://docs.ros.org/en/humble/Installation.html) for installation.

The deployment also depends on libraries such as ccache fmt spdlog eigen3. Execute the following command on the host computer for installation:

```bash
sudo apt update && sudo apt install -y ccache libfmt-dev libspdlog-dev libeigen3-dev
```

Then install the 5.10 real-time kernel for Orange Pi 5 Plus:

```bash
git clone https://github.com/Roboparty/atom01_deploy.git
cd atom01_deploy
git submodule update --init --recursive
cd assets
sudo apt install ./*.deb
cd ..
```

Next, grant the user permission to set real-time priorities:

```bash
sudo nano /etc/security/limits.conf
```

Add the following two lines at the end of the file, replacing orangepi with your actual username:

```bash
# Allow user 'orangepi' to set real-time priorities
orangepi   -   rtprio   98
orangepi   -   memlock  unlimited
```

Save, exit, and then reboot the Orange Pi for the settings to take effect.

## Hardware Connection

In the motor driver, can0 corresponds to the left leg, can1 corresponds to the right leg and waist, can2 corresponds to the left hand, and can3 corresponds to the right hand. By default, they are numbered according to the order of USB-to-CAN insertion into the host computer, with the first inserted being can0. It is recommended to plug the USB-to-CAN into the 3.0 interface of the host computer. If using a USB hub, please also use a 3.0 interface USB hub and plug it into the 3.0 interface. IMU and gamepad can be plugged into USB2.0 interfaces.

Write udev rules to bind USB ports with USB-to-CAN devices, so you don't need to care about the order of device insertion into the host computer. An example is in 99-auto-up-devs.rules. What needs to be modified is the KERNELS item, changing it to the KERNELS attribute item of the USB interface that the USB-to-CAN device wants to bind to. Input the following command on the host computer to monitor USB events:

```bash
sudo udevadm monitor
```

When inserting a device into the USB port, the KERNELS attribute item of that USB interface will be displayed, such as /devices/pci0000:00/0000:00:14.0/usb3/3-8. When matching the KERNELS attribute item, we can use 3-8. If you want to bind to a USB port on a hub connected to that USB interface, 3-8.x will appear, then use 3-8.x to match the USB port on the hub.

After writing, execute the following on the host computer:

```bash
sudo cp 99-auto-up-devs.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

After that, unplug all CAN devices and plug them into the specified USB interfaces to take effect.

The udev rules also include IMU serial port configuration, which also needs to modify the KERNELS item. The method is not repeated here. If the rules take effect normally, all CAN should be automatically configured and enabled. You can input the `ip a` command on the host computer to check the results.

If you don't configure udev rules, you need to insert USB-to-CAN devices in the order mentioned above and manually configure CAN and IMU serial ports:

```bash
sudo ip link set canX up type can bitrate 1000000
sudo ip link set canX txqueuelen 1000
# canX is can0 can1 can2 can3, you need to input the above two commands for each CAN
sudo chmod 666 /dev/ttyUSB0
```

## Software Usage
### Robot Startup

```bash
./tools/start_robot.sh
```

### Gamepad Control

- **A Button**: Initialize/Deinitialize motors
- **X Button**: Reset motors
- **B Button**: Start/Pause inference
- **Y Button**: Switch between Gamepad Control / cmd_vel Control
- **LB Button**: Switch policy mode (available in beyondmimic / interrupt modes). Pressing it will pause inference, and you need to manually press the B button to start inference.

### Service Interface

You can control the robot by calling ROS2 services via command line:

- **Initialize Motors**:
  ```bash
  ros2 service call /init_motors std_srvs/srv/Trigger
  ```

- **Deinitialize Motors**:
  ```bash
  ros2 service call /deinit_motors std_srvs/srv/Trigger
  ```

- **Start Inference**:
  ```bash
  ros2 service call /start_inference std_srvs/srv/Trigger
  ```

- **Stop Inference**:
  ```bash
  ros2 service call /stop_inference std_srvs/srv/Trigger
  ```

- **Clear Errors**:
  ```bash
  ros2 service call /clear_errors std_srvs/srv/Trigger
  ```

- **Set Zeros**:
  ```bash
  ros2 service call /set_zeros std_srvs/srv/Trigger
  ```

- **Reset Joints**:
  ```bash
  ros2 service call /reset_joints std_srvs/srv/Trigger
  ```

- **Refresh Joint States**:
  ```bash
  ros2 service call /refresh_joints std_srvs/srv/Trigger
  ```

## Python SDK

This repository provides a Python SDK to facilitate hardware control using Python scripts. Before use, please ensure you have sourced the ROS2 environment and the workspace `install/setup.bash`.

### 1. IMU SDK (`imu_py`)

#### Static Methods
- `create_imu(imu_id: int, interface_type: str, interface: str, imu_type: str, baudrate: int = 0) -> IMUDriver`: Create an IMU driver instance.

#### Member Methods
- `get_imu_id() -> int`: Get IMU ID.
- `get_ang_vel() -> List[float]`: Get angular velocity [x, y, z].
- `get_quat() -> List[float]`: Get quaternion [w, x, y, z].
- `get_lin_acc() -> List[float]`: Get linear acceleration [x, y, z].
- `get_temperature() -> float`: Get temperature.

#### Example
```python
import imu_py
imu = imu_py.IMUDriver.create_imu(8, "serial", "/dev/ttyUSB0", "HIPNUC", 921600)
quat = imu.get_quat()
```

### 2. Motor SDK (`motors_py`)

Provides `MotorControlMode` enum: `NONE`, `MIT`, `POS`, `SPD`.

#### Static Methods
- `create_motor(motor_id: int, interface_type: str, interface: str, motor_type: str, motor_model: int, master_id_offset: int = 0, motor_zero_offset: double = 0.0) -> MotorDriver`: Create a motor driver instance.

#### Member Methods
- `init_motor()`: Initialize motor.
- `deinit_motor()`: Deinitialize motor.
- `set_motor_control_mode(mode: MotorControlMode)`: Set control mode.
- `motor_mit_cmd(pos: float, vel: float, kp: float, kd: float, torque: float)`: MIT control command.
- `motor_pos_cmd(pos: float, spd: float, ignore_limit: bool = False)`: Position control command.
- `motor_spd_cmd(spd: float)`: Speed control command.
- `lock_motor() / unlock_motor()`: Lock/Unlock motor.
- `set_motor_zero()`: Set current position as zero point.
- `clear_motor_error()`: Clear error.
- `get_motor_pos() -> float`: Get position (rad).
- `get_motor_spd() -> float`: Get speed (rad/s).
- `get_motor_current() -> float`: Get current (A).
- `get_motor_temperature() -> float`: Get temperature (°C).
- `get_error_id() -> int`: Get error ID.
- `write_motor_flash()`: Write current parameters to Flash.
- `reset_motor_id(new_id: int)`: Reset motor ID.

#### Example
```python
import motors_py
motor = motors_py.MotorDriver.create_motor(1, "can", "can0", "DM", 0, 16)
motor.init_motor()
motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
motor.motor_mit_cmd(0.0, 0.0, 5.0, 1.0, 0.0)
```

### 3. Robot SDK (`robot_py`)

The `RobotInterface` class is used to unify the control of the entire robot, automatically loading motors and IMU by reading the configuration file.

#### Constructor
- `RobotInterface(config_file: str)`: Create an instance based on the configuration file path.

#### Member Methods
- `init_motors()`: Initialize all motors.
- `deinit_motors()`: Deinitialize all motors.
- `reset_joints(joint_default_angle: List[float])`: Reset all joints to default angles.
- `apply_action(action: List[float])`: Apply control action (joint target position/torque, etc., depending on internal implementation).
- `refresh_joints()`: Refresh all joint states.
- `set_zeros()`: Set all current joint positions to zero.
- `clear_errors()`: Clear all motor errors.
- `get_joint_q() -> List[float]`: Get all joint positions.
- `get_joint_vel() -> List[float]`: Get all joint velocities.
- `get_joint_tau() -> List[float]`: Get all joint torques.
- `get_quat() -> List[float]`: Get IMU quaternion [w, x, y, z].
- `get_ang_vel() -> List[float]`: Get IMU angular velocity.

#### Properties
- `is_init`: (Read-only) Whether the robot is initialized.

#### Example
```python
import robot_py
robot = robot_py.RobotInterface("config/robot.yaml")
robot.init_motors()
robot.apply_action([0.0] * 23)
```

**Tip**: For detailed Python script examples, please refer to the `scripts/` directory.

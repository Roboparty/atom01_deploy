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

- **Easy to Use**: Provides complete detailed code for learning and allows code modification.
- **Isolation**: Different functions are implemented by different packages, supporting the addition of custom function packages.
- **Long-term Support**: This repository will be updated along with the training repository code and will provide long-term support.

## Controller Connection

The deployment framework has been fully verified on **Orange Pi 5 Plus** and **RDK X5**.

- **Orange Pi 5 Plus**: OS is `Ubuntu 22.04`, kernel version is `5.10`
- **RDK X5**: OS is `Ubuntu 22.04`, kernel version is `6.1.83`

For controller connection methods and related resources, see [Orange Pi 5 Plus Wiki](http://www.orangepi.cn/orangepiwiki/index.php/Orange_Pi_5_Plus) and [RDK X5 Doc](https://d-robotics.github.io/rdk_doc/Quick_start/hardware_introduction/rdk_x5).

## Environment Setup

1. First install ROS2 Humble, refer to [ROS Official](https://docs.ros.org/en/humble/Installation.html) for installation.

2. The deployment also depends on libraries such as `ccache`, `fmt`, `spdlog`, `eigen3`. Execute the following instruction on the controller to install:

   ```bash
   sudo apt update && sudo apt install -y ccache libfmt-dev libspdlog-dev libeigen3-dev
   ```

3. Next, clone the deployment code:

   ```bash
   git clone https://github.com/Roboparty/atom01_deploy.git
   cd atom01_deploy
   git submodule update --init --recursive
   ```

4. If using Orange Pi 5 Plus, execute the following instructions to install the **5.10 real-time kernel**:
   
   > **Note**: For RDK X5, there is no need to perform this step. Please directly flash the image we provide that has the real-time kernel pre-installed.

   ```bash
   cd assets
   sudo apt install ./*.deb
   cd ..
   ```

5. Next, grant the user permission to set real-time priorities:

   ```bash
   sudo nano /etc/security/limits.conf
   ```

   Add the following two lines at the end of the file (**be sure to replace `orangepi` with your actual username**, for example, the default username for RDK X5 is `sunrise`):

   ```bash
   # Allow user 'orangepi' to set real-time priorities
   orangepi   -   rtprio   98
   orangepi   -   memlock  unlimited
   ```

   Restart the device to make the configuration take effect, and then verify it through the following command:

   ```bash
   ulimit -r
   ```

   > **Tip**: An output of **98** indicates a successful configuration.

## AP Configuration (Optional)

To facilitate debugging without an Ethernet cable and monitor, a WiFi Access Point (AP) can be enabled for the controller board. Configuration-related files are in the `tools/create_ap` directory.

> **Note**: Due to the limitation of a single network card, after enabling the AP mode, the built-in WiFi of the controller board will be difficult to connect to external networks such as a home router.
>
> - **If you need to connect to the external network to download packages or environments**, please **connect a wired network** to the controller board.
> - **If you temporarily want to restore wireless Internet access**, you can stop the service through the following command (requires a monitor or wired connection to log in):
>   ```bash
>   sudo systemctl stop create_ap.service
>   ```

1. Execute in the project root directory to install and grant permissions:

   ```bash
   sudo cp tools/create_ap/create_ap /usr/bin/
   sudo chmod +x /usr/bin/create_ap
   ```

2. Deploy systemd service file:

   ```bash
   sudo cp tools/create_ap/create_ap.service /etc/systemd/system/
   ```

3. Copy the configuration file according to your controller board:

   For **Orange Pi 5 Plus** please use this configuration:

   ```bash
   sudo cp tools/create_ap/create_ap_orangepi.conf /etc/create_ap.conf
   ```

   For **RDK X5** please use this configuration:

   ```bash
   sudo cp tools/create_ap/create_ap_sunrise.conf /etc/create_ap.conf
   ```

   > **Description**: Under the default configuration, the hotspot name (`SSID`) is **`atom`** and the password (`PASSPHRASE`) is **`jujujuju`**. To customize the hotspot name or password, you can edit the `/etc/create_ap.conf` file and modify the corresponding fields.

4. Enable autostart on boot and start the hotspot immediately:

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable create_ap.service
   sudo systemctl start create_ap.service
   ```

## Hardware Configuration

Before connecting, please complete the settings for motor ID, and IMU baud rate and frequency.

For the **motor ID**, please refer to the motor ID definition in the assembly manual, and use the Damiao host computer tool to set it. For tutorials, please see [Damiao Technology Docs](https://gitee.com/kit-miao/damiao-document).

For the **IMU**, we use **`921600` baud rate** and **`500HZ` frequency** by default. How to modify it using the host computer, see the [HiPNUC Product Manual](https://www.hipnuc.com/resource_hi14.html).
> **Tip**: Other baud rates can also be used, but please **ensure the frequency is greater than 200HZ**. If a different baud rate is used, synchronously modify the IMU configuration in `src/inference/config/robot.yaml`.

## Hardware Connection

The default CAN mapping relationship for motor drivers is as follows (numbered in the order USB-to-CAN is inserted into the controller, the first inserted is `can0`):
- **`can0`** corresponds to **Left leg**
- **`can1`** corresponds to **Right leg and waist**
- **`can2`** corresponds to **Left hand**
- **`can3`** corresponds to **Right hand**

> **Recommendation**: Plug the USB-to-CAN into the **USB 3.0 interface** of the controller. If using a USB hub, please also use a 3.0 interface hub and plug it into a 3.0 interface; IMU and gamepad can be plugged into USB 2.0 interfaces. For specific details, refer to the wiring instructions.

### Method 1: Manual Configuration (Not Recommended)
If you don't configure udev rules, you need to firmly follow the order above to insert USB-to-CAN, and after inserting the IMU, manually configure the CAN and IMU serial ports:

```bash
# CAN Configuration
sudo ip link set canX up type can bitrate 1000000
sudo ip link set canX txqueuelen 1000
# canX is can0, can1, can2, can3, you need to input the above two instructions for each can

# IMU Configuration
sudo chmod 666 /dev/ttyUSB0
```

### Method 2: Use udev rules for automatic binding (Recommended)
Write udev rules to physically bind USB interfaces to corresponding devices, so **you don't need to insert devices in order**. We provide examples `99-auto-up-devs-orangepi.rules` and `99-auto-up-devs-sunrise.rules`. If your wiring is exactly the same as the wiring instructions, you can use them directly.

If the wiring is inconsistent, you need to modify the `KERNELS` item in the file to correspond to the actually bound USB interface. Enter the following instruction on the controller to monitor USB events:

```bash
sudo udevadm monitor
```

When a device is inserted into the USB port, the terminal will display the `KERNELS` attribute item of that USB interface, such as `/devices/pci0000:00/0000:00:14.0/usb3/3-8`. Use `3-8` when matching the `KERNELS` attribute. If it's bound to a USB port on a hub connected to that interface, `3-8.x` will appear. In this case, use `3-8.x` to match the USB port on the hub.

After writing, execute on the controller:

```bash
# For RDK X5, use 99-auto-up-devs-sunrise.rules
sudo cp 99-auto-up-devs-orangepi.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

Restart the controller for it to take effect.

The udev rules also include the IMU serial port configuration. If the rules take effect normally, all CAN interfaces should automatically finish configuration and be enabled. You can check the results by entering the `ip a` command on the controller.

## Software Usage

### Start Software

> **Warning**: Before starting the robot, ensure the robot has completed zero point calibration. **Please be sure to read the safety operation guide!**

Additionally, pay special attention to the zero point offset configuration in `src/inference/config/robot.yaml`:

```yaml
motor_zero_offset: 
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.093,
     0.0, 0.0, 0.0, 0.0, 0.0,
     0.0 ,0.0, 0.0, 0.0, 0.0]
```

- If you calibrate by **turning the waist yaw to the limit block**: keep `2.093`.
- If you **use a 3D printed part to fix the waist yaw** for calibration: change `2.093` to `0.0`.

Once everything is ready, run the script to start the software:

```bash
./tools/start_robot.sh
```

### Gamepad Control

- **A Button**: Initialize/Deinitialize motors
- **X Button**: Reset motors
- **B Button**: Start/Pause inference
- **Y Button**: Switch between Gamepad Control / cmd_vel Control
- **LB Button**: Switch policy mode (available in beyondmimic / interrupt modes)

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

- **Read Joints**:

  ```bash
  ros2 service call /read_joints std_srvs/srv/Trigger
  ```

- **Read IMU**:

  ```bash
  ros2 service call /read_imu std_srvs/srv/Trigger
  ```

## Python SDK

This repository provides a Python SDK to facilitate hardware control using Python scripts. Before use, please ensure you have sourced the ROS2 environment and the workspace `install/setup.bash`.

> **Tip**: For detailed Python script examples, please refer to the `scripts/` directory.

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
- `get_motor_id() -> int`: Get motor ID.
- `get_motor_control_mode() -> int`: Get control mode.
- `get_response_count() -> int`: Get response count.
- `refresh_motor_status()`: Refresh motor status.

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

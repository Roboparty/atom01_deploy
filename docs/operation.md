# Common Operation

[Project Home](../README.md) | [中文](operation_CN.md)

These operations apply to RPO and RP1. Refer to [RPO Deployment](rpo_deploy.md) for wiring, calibration references, and policy selection. Run all commands from the repository root.

Follow the corresponding robot guide for environment setup, wiring, zeroing, and startup. See [RPO Environment Setup](rpo_deploy.md#environment-setup) for dependency installation and real-time configuration. Startup defaults to `rpo` / `default`; select RP1 explicitly when using the available files. Parameter definitions and file formats are in the [YAML Configuration Reference](configuration.md).

## ROS Environment and Build

`start_robot.sh` and `start_camera.sh` use `tools/ros_environment.sh` to select the ROS distribution automatically according to the following rules.

- If the terminal has `ROS_DISTRO` set, only `humble` and `jazzy` are accepted; other values cause an error.
- Without `ROS_DISTRO`, `/opt/ros/humble/setup.bash` is checked first, followed by Jazzy. If neither exists, startup fails.

For manual builds, services, or Python SDK use, load one distribution in a fresh terminal. For Humble:

```bash
source /opt/ros/humble/setup.bash
```

Or for Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

For the initial build or after source changes, build and load the workspace after loading ROS:

```bash
colcon build --symlink-install
source install/setup.bash
```

The workspace build generates `install/setup.bash`. For an existing build, load the corresponding ROS distribution and `install/setup.bash` in each new terminal. Humble example:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Both startup scripts build and load the environment within their own processes. Configure the calling terminal independently by loading ROS and the workspace before service calls or Python SDK use.

Use a single ROS distribution per terminal and workspace. When changing distributions, use a fresh terminal and rebuild in a separate workspace to maintain consistency between the environment and build artifacts. Completed build validation covers Humble.

## Startup and Logs

The startup command is `bash tools/start_robot.sh --robot ROBOT --policy POLICY`. Defaults are `rpo` / `default`; see the robot guides for specific commands.

RP1 `default` uses the trained Flat policy (3 history frames, 50 Hz). `beyondmimic` starts with the same Flat policy; press LB to switch between walking and waving. The waving policy uses one frame and holds the final reference pose:

```bash
bash tools/start_robot.sh rp1_24dof default
bash tools/start_robot.sh rp1_24dof beyondmimic
bash tools/start_robot.sh rp1_24dof amp
bash tools/start_robot.sh rp1_24dof interrupt
```

`amp` uses the RP1 AMP policy with three history frames and the training velocity bounds. `interrupt` uses the RP1 Interrupt policy with the same three-frame history plus the interrupt-mode flag; the existing RP1 arm-interrupt control path is used when interrupt mode is enabled.

Startup creates the `inference_session` and `joy_session` screen sessions. Inspect their output:

```bash
screen -r inference_session
screen -r joy_session
```

Inside screen, press Ctrl+A followed by D to detach the session while the background process continues running. Shut down in the following order: stop inference, disable motors, and terminate the background sessions.

```bash
ros2 service call /stop_inference std_srvs/srv/Trigger
ros2 service call /deinit_motors std_srvs/srv/Trigger
screen -S inference_session -X quit
screen -S joy_session -X quit
```

Camera startup uses `realsense_session` and `camera_session`; its prerequisites are described in the RPO `parkour` instructions.

## Gamepad Controls

| Control | Operation |
| --- | --- |
| X | Initialize / disable motors |
| A | Reset to the selected policy's default joint angles |
| B | Start / pause inference; initialize motors before starting |
| Y | Switch gamepad / `cmd_vel` control |
| LB | Switch modes for BeyondMimic / Interrupt; start or reset UFO latent playback |
| RB | Select a BeyondMimic motion sequence |
| Right stick | Forward/backward/lateral movement when supported by the policy |
| LT / RT | Left / right rotation when supported by the policy |

Motors remain enabled while inference is paused. Disable them with the X control or `/deinit_motors` service.

## Service Interfaces

Load the environment in the calling terminal and ensure the node is running. All services below use `std_srvs/srv/Trigger`:

| Service | Operation | Command |
| --- | --- | --- |
| `/init_motors` | Initialize and enable motors | `ros2 service call /init_motors std_srvs/srv/Trigger` |
| `/deinit_motors` | Disable motors | `ros2 service call /deinit_motors std_srvs/srv/Trigger` |
| `/start_inference` | Start inference | `ros2 service call /start_inference std_srvs/srv/Trigger` |
| `/stop_inference` | Stop inference | `ros2 service call /stop_inference std_srvs/srv/Trigger` |
| `/clear_errors` | Clear motor errors | `ros2 service call /clear_errors std_srvs/srv/Trigger` |
| `/set_zeros` | Set current motor-shaft positions as hardware zeros | `ros2 service call /set_zeros std_srvs/srv/Trigger` |
| `/reset_joints` | Reset to the selected policy's default pose | `ros2 service call /reset_joints std_srvs/srv/Trigger` |
| `/refresh_joints` | Refresh motor status through each driver and update joint caches | `ros2 service call /refresh_joints std_srvs/srv/Trigger` |
| `/read_joints` | Update and read joint states | `ros2 service call /read_joints std_srvs/srv/Trigger` |
| `/read_imu` | Update and read IMU state | `ros2 service call /read_imu std_srvs/srv/Trigger` |

For example:

```bash
ros2 service call /read_joints std_srvs/srv/Trigger
```

The `/set_zeros` service requires initialized motors, stopped inference, and the correct physical calibration reference. It sends hardware-zero commands while retaining the YAML software offsets. After zeroing, power-cycle the motors before using the new zero points. For initial manual calibration, use the robot's matching `scripts/config/set_zero_*.yaml` and verify the physical reference before writing zero points.

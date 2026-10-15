# 通用操作

[项目首页](../README_CN.md) | [English](operation.md)

以下操作适用于 RPO 与 RP1。硬件接线、零位基准和策略说明见 [RPO 部署](rpo_deploy_CN.md)。所有命令均在仓库根目录执行。

环境准备、硬件接线、电机标零和软件启动按对应机器人的部署文档执行。RPO 的依赖安装与实时配置见 [RPO 环境准备](rpo_deploy_CN.md#环境准备)。启动默认使用 `rpo` / `default`，使用 RP1 文件时须显式选择机器人。参数定义与文件格式见 [YAML 配置参考](configuration_CN.md)。

## ROS 环境与构建

`start_robot.sh` 和 `start_camera.sh` 通过 `tools/ros_environment.sh` 自动选择 ROS 发行版，选择规则如下：

- 当前终端已设置 `ROS_DISTRO`：支持值为 `humble` 和 `jazzy`，其他值触发错误并终止启动。
- 未设置 `ROS_DISTRO`：优先查找 `/opt/ros/humble/setup.bash`，其次 Jazzy；两个环境文件均不存在时，终止启动并报告错误。

需要手动构建、调用服务或运行 Python SDK 时，在新终端加载所用版本。Humble：

```bash
source /opt/ros/humble/setup.bash
```

或 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
```

首次使用或代码更新后，完成 ROS 环境加载并构建工作空间：

```bash
colcon build --symlink-install
source install/setup.bash
```

`install/setup.bash` 由工作空间构建生成。已完成构建的工作空间，在新终端中加载对应 ROS 发行版和 `install/setup.bash`。Humble 示例：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

两个启动脚本在自身进程中完成编译与环境加载。调用终端的环境需独立配置，执行服务调用或 Python SDK 前应加载 ROS 与工作空间环境。

每个终端和工作空间应使用单一 ROS 发行版。切换发行版时，使用新终端和独立工作空间重新构建，以确保环境与构建产物一致。当前已完成的构建验证范围为 Humble。

## 启动与后台日志

启动命令为 `bash tools/start_robot.sh --robot ROBOT --policy POLICY`。省略参数时使用 `rpo` / `default`；具体命令见对应机器人文档。

RP1 的 `default` 使用已训练的 Flat 策略（3 帧历史，50 Hz）。`beyondmimic` 启动时使用同一 Flat 策略，按 LB 在行走和挥手之间切换。挥手策略使用 1 帧观测，动作结束后保持末帧：

```bash
bash tools/start_robot.sh rp1_24dof default
bash tools/start_robot.sh rp1_24dof beyondmimic
bash tools/start_robot.sh rp1_24dof amp
bash tools/start_robot.sh rp1_24dof interrupt
```

`amp` 使用 RP1 AMP 策略，保留 3 帧历史和训练时的速度指令范围。`interrupt` 使用 RP1 Interrupt 策略，同样保留 3 帧历史，并额外输入干扰模式标志；按 LB 进入或退出干扰模式后，部署节点沿用现有 RP1 手臂干扰控制接口。

启动后创建 `inference_session` 和 `joy_session` 两个 screen 会话。查看输出：

```bash
screen -r inference_session
screen -r joy_session
```

在 screen 中依次按 Ctrl+A、D 分离会话，后台进程持续运行。停机顺序为停止推理、失能电机、终止后台会话：

```bash
ros2 service call /stop_inference std_srvs/srv/Trigger
ros2 service call /deinit_motors std_srvs/srv/Trigger
screen -S inference_session -X quit
screen -S joy_session -X quit
```

相机使用 `realsense_session` 和 `camera_session`；启动前置条件见 RPO 的 `parkour` 说明。

## 手柄控制

| 按键 | 操作 |
| --- | --- |
| X | 初始化 / 失能电机 |
| A | 复位到所选策略的默认关节角度 |
| B | 开始 / 暂停推理，开始前需初始化电机 |
| Y | 切换手柄控制 / `cmd_vel` 控制 |
| LB | BeyondMimic / Interrupt 切换策略模式；UFO 启动或重置 latent 播放 |
| RB | BeyondMimic 切换动作序列 |
| 右摇杆 | 策略支持时控制前后左右移动 |
| LT / RT | 策略支持时控制转向（左 / 右旋转） |

推理暂停后，电机保持使能状态；失能通过 X 按键或 `/deinit_motors` 服务执行。

## 服务接口

调用前在当前终端加载环境，并确认相应节点已启动。以下服务均使用 `std_srvs/srv/Trigger`：

| 服务 | 操作 | 调用命令 |
| --- | --- | --- |
| `/init_motors` | 初始化并使能电机 | `ros2 service call /init_motors std_srvs/srv/Trigger` |
| `/deinit_motors` | 失能电机 | `ros2 service call /deinit_motors std_srvs/srv/Trigger` |
| `/start_inference` | 开始推理 | `ros2 service call /start_inference std_srvs/srv/Trigger` |
| `/stop_inference` | 停止推理 | `ros2 service call /stop_inference std_srvs/srv/Trigger` |
| `/clear_errors` | 清除电机错误 | `ros2 service call /clear_errors std_srvs/srv/Trigger` |
| `/set_zeros` | 将当前电机轴位置设为硬件零点 | `ros2 service call /set_zeros std_srvs/srv/Trigger` |
| `/reset_joints` | 复位到所选策略的默认姿态 | `ros2 service call /reset_joints std_srvs/srv/Trigger` |
| `/refresh_joints` | 按驱动刷新电机状态并更新关节缓存 | `ros2 service call /refresh_joints std_srvs/srv/Trigger` |
| `/read_joints` | 更新并读取关节状态 | `ros2 service call /read_joints std_srvs/srv/Trigger` |
| `/read_imu` | 更新并读取 IMU 状态 | `ros2 service call /read_imu std_srvs/srv/Trigger` |

例如：

```bash
ros2 service call /read_joints std_srvs/srv/Trigger
```

`/set_zeros` 的调用条件为电机已初始化、推理已停止且机器人位于正确的物理标零基准。该服务发送硬件标零命令，YAML 软件零偏保持原值。标零结束后须重新给电机上下电，再使用新的零点。首次人工标零应使用对应机器人的 `scripts/config/set_zero_*.yaml`，零点写入前须确认物理基准。

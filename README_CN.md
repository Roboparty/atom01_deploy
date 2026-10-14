# roboparty_deploy

[![许可证：GPL v3](https://img.shields.io/badge/许可证-GPLv3-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
![Linux](https://img.shields.io/badge/Linux-x86_64%20%7C%20aarch64-orange)

[English](README.md) | [中文](README_CN.md)

## 仓库介绍

`roboparty_deploy` 是面向 RPO 和 RP1 的 ROS 2 部署框架，包含电机驱动、IMU、策略推理和深度感知模块。

仓库采用模块化架构，提供完整的部署代码，支持自定义功能包扩展。

开源地址：[Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy)。维护者：RoboParty；支持渠道：GitHub Issues。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [RPO 部署](docs/rpo_deploy_CN.md) | RPO 主控连接、环境准备、实时内核、AP 配置、接线、标零及策略和相机启动 |
| RP1 部署 | Coming soon（即将发布） |
| [YAML 配置参考](docs/configuration_CN.md) | 配置文件加载关系、硬件、推理、相机和标零参数，以及排列顺序和约束 |
| [通用操作](docs/operation_CN.md) | 构建、ROS 自动检测、日志、手柄和服务 |
| [Python SDK](docs/python_sdk_CN.md) | IMU、电机、机器人接口，以及示例脚本适用范围 |

## 项目架构

| 路径 | 职责 |
| --- | --- |
| `src/motors` | DM、EVO、LRO、RP、XYN 电机驱动与通信 |
| `src/imu` | HIPNUC、MCT7123 IMU 驱动 |
| `src/inference` | 机器人接口、ONNX 策略推理、观测组装和动作数据读取 |
| `src/inference/robots` | 各机器人的硬件配置、策略参数、模型与动作数据 |
| `src/camera` | RealSense 接入、深度处理与观测发布 |
| `tools` | 环境检测及机器人、相机启动工具 |
| `scripts` | Python SDK 示例和电机标零工具 |
| `docs` | 环境、部署、配置与接口文档 |

推理模块通过机器人接口读取电机和 IMU 状态，将策略输出转换为关节目标，由电机驱动执行。相机模块独立处理深度图，并通过 ROS 话题向推理模块提供感知观测。

## 许可证

除另有说明外，本仓库中由 RoboParty 创作的原创内容依据 [GNU General Public License v3.0 only](LICENSE)（`GPL-3.0-only`）授权。

Git 子模块、已打包二进制文件及其他第三方资料继续遵循其各自的版权和许可证条款；仓库级 GPLv3 声明不会替代这些条款。当前许可证清单和再分发说明见 [LICENSES.md](LICENSES.md)。

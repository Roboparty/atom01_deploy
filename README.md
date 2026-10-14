# roboparty_deploy

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
![Linux](https://img.shields.io/badge/Linux-x86_64%20%7C%20aarch64-orange)

[English](README.md) | [中文](README_CN.md)

## Repository Overview

`roboparty_deploy` is a ROS 2 deployment framework for RPO and RP1, with motor drivers, IMU interfaces, policy inference, and depth perception.

The repository provides complete deployment code with a modular architecture that supports custom packages.

Repository: [Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy). Maintainer: RoboParty. Support: GitHub Issues.

## Documentation

| Guide | Contents |
| --- | --- |
| [RPO Deployment](docs/rpo_deploy.md) | RPO controller connection, environment, real-time kernel, AP setup, wiring, zeroing, and policy/camera startup |
| RP1 Deployment | Coming soon |
| [YAML Configuration Reference](docs/configuration.md) | Configuration loading, hardware, inference, camera, and zeroing parameters, ordering, and constraints |
| [Common Operation](docs/operation.md) | Build, automatic ROS detection, logs, gamepad, and services |
| [Python SDK](docs/python_sdk.md) | IMU, motor, and robot interfaces, plus example-script compatibility |

## Project Architecture

| Path | Responsibility |
| --- | --- |
| `src/motors` | DM, EVO, LRO, RP, and XYN motor drivers and communication |
| `src/imu` | HIPNUC and MCT7123 IMU drivers |
| `src/inference` | Robot interface, ONNX inference, observation assembly, and motion data loading |
| `src/inference/robots` | Hardware settings, policy parameters, models, and motion data for each robot |
| `src/camera` | RealSense integration, depth processing, and observation publishing |
| `tools` | Environment detection and robot/camera startup tools |
| `scripts` | Python SDK examples and motor zeroing tools |
| `docs` | Environment, deployment, configuration, and interface documentation |

Inference reads motor and IMU state through the robot interface, converts policy output into joint targets, and sends them through motor drivers. The camera module processes depth independently and supplies perception observations over ROS topics.

## License

Unless otherwise stated, original content authored by RoboParty in this repository is licensed under the [GNU General Public License v3.0 only](LICENSE) (`GPL-3.0-only`).

Git submodules, packaged binaries, and other third-party materials retain their own copyright and license terms. The repository-level GPLv3 statement does not replace those terms. See [LICENSES.md](LICENSES.md) for the current license inventory and redistribution notes.

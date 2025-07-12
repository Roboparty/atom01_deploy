# Deploy ROS2

## ENV

部署环境需要安装ccache fmt spdlog eigen3，均可使用apt安装，自行询问agent，不再赘述。

电机驱动中can0对应左腿，can1对应右腿加腰，can2对应左手，can3对应右手，默认按照usb转can插入上位机顺序编号，先插入的是can0。

编写udev规则用来将USB口与usb转can绑定，即可不需要再管插入上位机顺序，示例在96-auto-up-devs.rules中，需要修改的是KERNELS项，将其修改为该usb转can想要对应绑定的USB接口的KERNELS属性项。编写完成后：

```bash
sudo cp 96-auto-up-devs.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

即可生效，如果发现usb转can和USB口绑定失败，说明KERNELS项配置有问题，请询问agent获得更多帮助。

该udev规则还包括IMU串口配置，也需要修改KERNELS项，不再赘述。如果规则正常生效，can应该全部自动配置完毕并使能，IMU串口权限也自动修改完毕。

如果实在配不好udev规则，只好按照上文顺序插入usb转can，并手动配置can和IMU串口：

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
# 其他can同理

sudo chmod 666 /dev/ttyUSB0
```

## IMU

编译：

```bash
cd imu
colcon build --symlink-install --cmake-args -G Ninja
```

启动：

```bash
source install/setup.bash
ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

如果IMU串口权限正常，使用plotjunggler或者ros2 topic echo可以看到IMU数据。

## MOTORS

首先修改参数文件，保证参数正确！！！

编译：

```bash
cd motors
colcon build --symlink-install --cmake-args -G Ninja
```

启动：

```bash
source install/setup.bash
ros2 launch motors motors_spec_msg.launch.py
```

如果can配置正常，此时所有电机绿灯亮起。首先打开plotjunggler，订阅电机state话题后输入：

```bash
ros2 service call /read_motors motors/srv/ReadMotors
```

此时在plotjunggler中可以看到各个电机此时位置，保证没有反向关节且都在零点附近后输入：

```bash
ros2 service call /reset_motors motors/srv/ResetMotors
```

进行电机归零。

## INFERENCE

首先修改参数文件，保证参数正确！！！

编译：

```bash
cd inference
colcon build --symlink-install --cmake-args -G Ninja
```

保证IMU启动、电机启动且正常归零后，启动：

```bash
source install/setup.bash
ros2 launch inference inference.launch.py
```

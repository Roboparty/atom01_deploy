# Deploy ROS2

## 环境配置

部署依赖ccache fmt spdlog eigen3 ninja-build，在上位机中执行指令进行安装：

```bash
sudo apt update && sudo apt install -y ccache libfmt-dev libspdlog-dev libeigen3-dev ninja-build
```

电机驱动中can0对应左腿，can1对应右腿加腰，can2对应左手，can3对应右手，默认按照usb转can插入上位机顺序编号，先插入的是can0。

编写udev规则用来将USB口与usb转can绑定，即可不需要再管设备插入上位机顺序，示例在99-auto-up-devs.rules中，需要修改的是KERNELS项，将其修改为该usb转can想要对应绑定的USB接口的KERNELS属性项。在上位机输入指令监视USB事件：

```bash
sudo udevadm monitor
```

在USB上插入设备时就会显示该USB接口的KERNELS属性项，如/devices/pci0000:00/0000:00:14.0/usb3/3-8/3-8:1.1，我们在匹配KERNELS属性项时使用3-8即可。如果想要绑定在该USB接口上的扩展坞上的USB口则会有3-8-x出现，此时使用3-8-x进行匹配扩展坞上的USB口。

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

如果can配置正常，此时所有电机绿灯亮起。如果还未配置电机零点，将标定件插入把电机摆至零点后输入:

```bash
ros2 service call /ser_zeros motors/srv/SetZeros
```

观察到电机绿灯一个个灭下说明正在标零。

标零完成后重新启动motors并打开plotjunggler，订阅电机state话题后输入：

```bash
ros2 service call /read_motors motors/srv/ReadMotors
```

此时在plotjunggler中可以看到各个电机此时位置，保证没有反向关节且都在零点附近后输入：

```bash
ros2 service call /reset_motors motors/srv/ResetMotors
```

进行电机归零。

## INFERENCE

首先修改参数文件，保证参数正确！！！将训练得到的onnx模型放到models文件夹中。

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

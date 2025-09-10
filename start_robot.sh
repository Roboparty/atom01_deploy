#!/bin/bash

# 颜色定义，用于美化输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # 无颜色

# 函数：打印成功消息
print_success() {
    echo -e "${GREEN}$1${NC}"
}

# 函数：打印提示消息
print_info() {
    echo -e "${YELLOW}$1${NC}"
}

# 函数：打印错误消息
print_error() {
    echo -e "${RED}$1${NC}"
}

# 检查是否已安装screen
if ! command -v screen &> /dev/null; then
    print_info "未安装screen，正在安装..."
    sudo apt update && sudo apt install -y screen || {
        print_error "screen安装失败"
        exit 1
    }
fi

# 检查是否已source setup文件
if [ -z "$AMENT_PREFIX_PATH" ]; then
    print_info "未检测到ROS 2环境，正在执行source..."
    source install/setup.bash || {
        print_error "无法source install/setup.bash，请检查路径是否正确"
        exit 1
    }
fi

# 编译IMU包
print_info "编译IMU包..."
cd imu || {
    print_error "找不到imu目录"
    exit 1
}
colcon build --symlink-install || {
    print_error "IMU包编译失败"
    exit 1
}
source install/setup.bash
cd ..

# 编译电机包
print_info "编译电机包..."
cd motors || {
    print_error "找不到motors目录"
    exit 1
}
colcon build --symlink-install || {
    print_error "电机包编译失败"
    exit 1
}
source install/setup.bash
cd ..

# 编译推理包
print_info "编译推理包..."
cd inference || {
    print_error "找不到inference目录"
    exit 1
}
colcon build --symlink-install || {
    print_error "推理包编译失败"
    exit 1
}
source install/setup.bash
cd ..

# 停止可能正在运行的screen会话
print_info "停止现有相关screen会话..."
screen -S imu_session -X quit 2>/dev/null
screen -S motor_session -X quit 2>/dev/null
screen -S inference_session -X quit 2>/dev/null
screen -S joy_session -X quit 2>/dev/null

# 启动IMU (在screen后台运行)
print_info "启动IMU..."
screen -dmS imu_session bash -c "source install/setup.bash; ros2 launch hipnuc_imu imu_spec_msg.launch.py; exec bash"
sleep 5

# 检查IMU是否启动成功
if ! screen -list | grep -q "imu_session"; then
    print_error "IMU启动失败！"
    exit 1
fi
print_success "IMU启动成功 (screen会话: imu_session)"

# 启动电机
print_info "启动电机..."
screen -dmS motor_session bash -c "source install/setup.bash; ros2 launch motors motors_spec_msg.launch.py; exec bash"
sleep 5

# 检查电机是否启动成功
if ! screen -list | grep -q "motor_session"; then
    print_error "电机启动失败！"
    screen -S imu_session -X quit
    exit 1
fi
print_success "电机启动成功 (screen会话: motor_session)"

# 电机归零
print_info "正在进行电机归零..."
ros2 service call /reset_motors motors/srv/ResetMotors "{}"
if [ $? -ne 0 ]; then
    print_error "电机归零失败！"
    screen -S imu_session -X quit
    screen -S motor_session -X quit
    exit 1
fi
print_success "电机归零完成"
sleep 2

# # 启动推理模块
# print_info "启动推理模块..."
# screen -dmS inference_session bash -c "source install/setup.bash; ros2 launch inference inference.launch.py; exec bash"
# sleep 5

# # 检查推理模块是否启动成功
# if ! screen -list | grep -q "inference_session"; then
#     print_error "推理模块启动失败！"
#     screen -S imu_session -X quit
#     screen -S motor_session -X quit
#     exit 1
# fi
# print_success "推理模块启动成功 (screen会话: inference_session)"

# 启动手柄控制
print_info "启动手柄控制..."
screen -dmS joy_session bash -c "source install/setup.bash; ros2 run joy joy_node; exec bash"
sleep 2

# 检查手柄控制是否启动成功
if ! screen -list | grep -q "joy_session"; then
    print_error "手柄控制启动失败！"
    screen -S imu_session -X quit
    screen -S motor_session -X quit
    # screen -S inference_session -X quit
    exit 1
fi
print_success "手柄控制启动成功 (screen会话: joy_session)"

# 所有组件启动完成
print_success "----------------------------------------"
print_success "所有组件已在后台成功启动！"
print_success "使用以下命令查看各组件输出："
print_success "IMU: screen -r imu_session"
print_success "电机: screen -r motor_session"
# print_success "推理模块: screen -r inference_session"
print_success "手柄控制: screen -r joy_session"
print_success "----------------------------------------"
print_info "若要退出某个screen会话，按Ctrl+A然后按D"
print_info "使用以下命令停止所有组件："
print_info "screen -S imu_session -X quit"
print_info "screen -S motor_session -X quit"
# print_info "screen -S inference_session -X quit"
print_info "screen -S joy_session -X quit"


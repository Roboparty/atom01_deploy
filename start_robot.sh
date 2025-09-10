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

# 函数：启动组件并检查
start_component() {
    local session_name=$1
    local launch_cmd=$2
    local node_name=$3
    local sleep_time=$4

    print_info "启动 $session_name..."
    screen -dmS $session_name bash -c "$launch_cmd; exec bash"
    sleep $sleep_time

    if ! ros2 node list | grep -q "$node_name"; then
        print_error "$session_name 启动失败！未检测到 $node_name 节点。"
        cleanup_sessions
        exit 1
    fi
    print_success "$session_name 启动成功 (screen会话: $session_name)"
}

# 函数：清理所有会话
cleanup_sessions() {
    screen -S imu_session -X quit 2>/dev/null
    screen -S motor_session -X quit 2>/dev/null
    screen -S inference_session -X quit 2>/dev/null
    screen -S joy_session -X quit 2>/dev/null
}

# 切换到脚本目录
cd "$(dirname "$0")"

# 检查 colcon 和 ros2
if ! command -v colcon &> /dev/null; then
    print_error "colcon 未安装，请安装 ROS 2 开发工具"
    exit 1
fi
if ! command -v ros2 &> /dev/null; then
    print_error "ros2 未安装"
    exit 1
fi

# 检查是否已安装screen
if ! command -v screen &> /dev/null; then
    print_error "screen 未安装"
    exit 1
fi

# 检查是否已source setup文件
if [ -z "$AMENT_PREFIX_PATH" ]; then
    print_info "未检测到ROS 2环境，正在执行source..."
    source /opt/ros/humble/setup.bash || {
        print_error "无法source /opt/ros/humble/setup.bash，请检查路径是否正确"
        exit 1
    }
fi

# 编译IMU包
print_info "编译IMU包..."
cd imu || {
    print_error "找不到imu目录"
    exit 1
}
colcon build --symlink-install --cmake-args -G Ninja|| {
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
colcon build --symlink-install --cmake-args -G Ninja|| {
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
colcon build --symlink-install --cmake-args -G Ninja|| {
    print_error "推理包编译失败"
    exit 1
}
source install/setup.bash
cd ..

# 停止可能正在运行的screen会话
print_info "停止现有相关screen会话..."
cleanup_sessions

start_component "imu_session" "ros2 launch hipnuc_imu imu_spec_msg.launch.py" "/IMU_publisher" 2
start_component "motor_session" "ros2 launch motors motors_spec_msg.launch.py" "/MotorsNode" 2
start_component "inference_session" "ros2 launch inference inference.launch.py" "/Inference" 2
start_component "joy_session" "ros2 run joy joy_node" "/joy_node" 2

# 所有组件启动完成
print_success "----------------------------------------"
print_success "所有组件已在后台成功启动！"
print_success "使用以下命令查看各组件输出："
print_success "IMU: screen -r imu_session"
print_success "电机: screen -r motor_session"
print_success "推理模块: screen -r inference_session"
print_success "手柄控制: screen -r joy_session"
print_success "----------------------------------------"
print_info "若要退出某个screen会话，按Ctrl+A然后按D"
print_info "使用以下命令停止所有组件："
print_info "screen -S imu_session -X quit"
print_info "screen -S motor_session -X quit"
print_info "screen -S inference_session -X quit"
print_info "screen -S joy_session -X quit"
print_success "----------------------------------------"
print_info "按下X使能/失能电机"
print_info "按下A复位电机"
print_info "按下B开始推理"
print_info "按下Y读取电机数据"

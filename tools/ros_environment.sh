#!/bin/bash

setup_ros_environment() {
    local selected=auto
    case "$selected" in
        auto)
            if [ -n "${ROS_DISTRO:-}" ]; then
                selected=$ROS_DISTRO
            elif [ -f /opt/ros/humble/setup.bash ]; then
                selected=humble
            elif [ -f /opt/ros/jazzy/setup.bash ]; then
                selected=jazzy
            else
                print_error "未找到 ROS 2 Humble 或 Jazzy"
                return 1
            fi
            ;;
    esac
    case "$selected" in
        humble|jazzy) ;;
        *)
            print_error "当前 ROS 版本 $selected 不受支持，请在新终端选择 Humble 或 Jazzy"
            return 1
            ;;
    esac
    if [ -n "${ROS_DISTRO:-}" ] && [ "$ROS_DISTRO" != "$selected" ]; then
        print_error "当前环境为 $ROS_DISTRO，请在未加载 ROS 的新终端选择 $selected"
        return 1
    fi
    local setup_file="/opt/ros/$selected/setup.bash"
    if [ ! -f "$setup_file" ]; then
        print_error "ROS 2 $selected 未安装: $setup_file"
        return 1
    fi
    source "$setup_file" || return 1
    print_info "选择 ROS 2 版本: $selected"
}

#!/bin/bash

APP_NAME="rbq_description"
SIM_MODE=false
ROS2_DIR="ros2"

print_help() {
    echo "Usage: bash scripts/start_vision.bash [OPTIONS]"
    echo "Options:"
    echo "  --help      Display this help message and exit."
    echo "  --sim       Run in simulator mode."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) SIM_MODE=true; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    sleep 10
    exit 1
fi
if pgrep -x $APP_NAME > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi

export CYCLONEDDS_URI=file://$PWD/configs/cyclonedds_ros2.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash

cd $ROS2_DIR
colcon build --symlink-install
source install/setup.bash
echo -ne "\033]0;$APP_NAME\007"

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        ros2 launch rbq_description description.launch.py
    fi
    sleep 2
done

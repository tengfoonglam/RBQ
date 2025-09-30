#!/bin/bash

APP_NAME="rbq_driver"
SIM_MODE=false

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


if [ "$SIM_MODE" = true ]; then
    if [ ! "$EUID" -eq 0 ]; then
        echo "Run this script with sudo. Exiting..."
        sleep 10
        exit 1
    fi
else
    if [ "$EUID" -eq 0 ]; then
        echo "Do not run this script with sudo. Exiting..."
        sleep 10
        exit 1
    fi
fi

if pgrep -x $APP_NAME > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi

function set_terminal_title {
    echo -ne "\033]0;$1\007"
}
set_terminal_title "$APP_NAME"

cd ros2

if [ -d "build" ] && [ -d "install" ]; then
    echo "Build and install directories already exist, skipping colcon build..."
else
    echo "Building ROS2 packages..."
    
    colcon build --symlink-install
fi

source ./install/setup.bash

# Run loop
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        if [ "$SIM_MODE" = true ]; then
            echo "Starting ROS2 driver in simulation mode..."
            ros2 run rbq_driver rbq_driver -s
        else
            echo "Starting ROS2 driver..."
            ros2 run rbq_driver rbq_driver
        fi
    fi
    sleep 2
done

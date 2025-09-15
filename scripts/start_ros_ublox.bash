#!/bin/bash

APP_NAME="ublox_gps_node"

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
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

source ros2/install/setup.bash

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
	ros2 launch ublox_gps ublox_gps_node_zedf9p-launch.py
    fi
    sleep 2
done

#!/bin/bash

APP_NAME="rqt"

export CYCLONEDDS_URI=file://$PWD/configs/cyclonedds_ros2.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

echo -ne "\033]0;$APP_NAME\007"

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
            echo "Starting $APP_NAME..."
            $APP_NAME
    fi
    sleep 2
done

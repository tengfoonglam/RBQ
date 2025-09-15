#!/bin/bash

RBQ_DRIVER_ARGS=""
if [[ $* == *"-s"* ]]; then
    RBQ_DRIVER_ARGS="-s"
fi

echo "🚗 rbq_driver를 시작합니다..."
ros2 run rbq_driver rbq_driver $RBQ_DRIVER_ARGS &
sleep 1

echo "📡 livox_ros_driver2를 시작합니다..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 1

echo "🗺️ fast_lio mapping을 시작합니다..."
ros2 launch fast_lio mapping.launch.py config_file:=rbq_mid360.yaml rviz:=false &
sleep 1

echo "🤖 rbq_description slamDescription을 시작합니다..."
ros2 launch rbq_description SlamDescription.launch.py &
sleep 1

echo "✅ 모든 SLAM 컴포넌트가 시작되었습니다!"
echo "🛑 Ctrl+C를 눌러서 모든 프로세스를 종료할 수 있습니다."

wait

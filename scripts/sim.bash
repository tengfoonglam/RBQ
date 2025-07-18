#!/bin/bash

SIM_TYPE="mujoco"
ROS_ENABLED=false
VISION_ENABLED=false

print_help() {
    echo "Usage: bash scripts/sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message and exit."
    echo "  --vision        Run vision modules."
    echo "  --ros           Run ROS2 driver."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --vision) VISION_ENABLED=true; shift ;;
        --ros) ROS_ENABLED=true; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

gnome-terminal --tab --title="Motion"   -- bash -i -c "bash scripts/start_motion.bash --sim"
#gnome-terminal --tab --title="Network"  -- bash -i -c "bash scripts/start_network.bash --sim"

sleep 1

if [ "$VISION_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="Vision" -- bash -i -c "bash scripts/start_vision.bash --sim"
    gnome-terminal --tab --title="Mujoco" -- bash -i -c "bash scripts/start_mujoco.bash --vision"
else
    gnome-terminal --tab --title="Mujoco" -- bash -i -c "bash scripts/start_mujoco.bash"
fi

if [ "$ROS_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="rbq_driver"      -- bash -i -c "bash scripts/start_ros_driver.bash --sim"
    gnome-terminal --tab --title="rbq_description" -- bash -i -c "bash scripts/start_rviz.bash"
fi

gnome-terminal --tab --title="GUI" -- bash -i -c "bash scripts/start_gui.bash --sim"

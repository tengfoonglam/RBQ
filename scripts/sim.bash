#!/bin/bash

ROS_ENABLED=false
VISION_ENABLED=false
ROBOT_ARGS=()

print_help() {
    echo "Usage: bash scripts/sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message and exit."
    echo "  --vision        Run vision modules."
    echo "  --ros           Run ROS2 driver."
    echo "  --rb1           RB1 Arm enabled. DEFAULT false"
    echo "  --lims_ex       LIMS_EX enabled. DEFAULT false"
    echo "  --wheel         RBQ WHEEL enabled. DEFAULT false"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --vision) VISION_ENABLED=true; shift ;;
        --ros) ROS_ENABLED=true; shift ;;
        --rb1) ROBOT_ARGS+=("--rb1"); shift ;;
        --lims_ex) ROBOT_ARGS+=("--lims_ex"); shift ;;
        --wheel) ROBOT_ARGS+=("--wheel"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# if [ "$EUID" -eq 0 ]; then
#     echo "Do not run this script with sudo. Exiting..."
#     exit 1
# fi

gnome-terminal --tab --title="Motion"   -- bash -i -c "bash scripts/start_motion.bash --sim ${ROBOT_ARGS[@]}"
sleep 1
if [ "$VISION_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="Vision" -- bash -i -c "bash scripts/start_vision.bash --sim"
    gnome-terminal --tab --title="Mujoco" -- bash -i -c "bash scripts/start_mujoco.bash ${ROBOT_ARGS[@]} --vision "
    gnome-terminal --tab --title="mediamtx" -- bash -i -c "bash scripts/start_mediamtx.bash"
else
    gnome-terminal --tab --title="Mujoco" -- bash -i -c "bash scripts/start_mujoco.bash ${ROBOT_ARGS[@]}"
fi

if [ "$ROS_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="rbq_ros_driver " -- bash -i -c "bash scripts/start_ros_driver.bash --sim"
    gnome-terminal --tab --title="rbq_description" -- bash -i -c "bash scripts/start_rviz.bash --sim"
fi

gnome-terminal --tab --title="GUI" -- bash -i -c "bash scripts/start_gui.bash --sim ${ROBOT_ARGS[@]}"

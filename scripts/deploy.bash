#!/bin/bash

REMOTE_DEVICE="rbq@192.168.0.10"
REMOTE_HOME_DIR="~/rbq_ws"
REMOTE_BIN_DIR="$REMOTE_HOME_DIR/bin"
REMOTE_SCRIPTS_DIR="$REMOTE_HOME_DIR/scripts"
REMOTE_RESOURCES_DIR="$REMOTE_HOME_DIR/resources"
REMOTE_RBQ_WS_DIR="$REMOTE_HOME_DIR/.."
REMOTE_ROS2_SRC_DIR="$REMOTE_HOME_DIR/ros2/src"
REMOTE_CONFIGS_DIR="$REMOTE_HOME_DIR/configs"
BINARY_DIR="bin"
SCRIPTS_DIR="scripts"
RESOURCES_MEDIAMTX_DIR="resources/mediamtx"
ROS2_SRC_DIR="ros2/src"
LOGFILE="deploy.log"

SELECTED_BINARIES=()

print_help() {
    echo "Usage: ./deploy.bash [OPTIONS]"
    echo "  --help             Display this help message and exit."
    echo "  --pro [FILES...]    Specify binary names for selective deployment."
    echo "  --device [USER@IP] Set the remote device (default: $REMOTE_DEVICE)."
}

exec > >(tee -a "$LOGFILE") 2>&1

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --device) shift; REMOTE_DEVICE="$1"; shift ;;
        --pro) shift; while [[ $# -gt 0 && ! $1 =~ ^- ]]; do SELECTED_BINARIES+=("$1"); shift; done ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ ! -d "$BINARY_DIR" ]; then
    echo -e "\e[31mError: bin directory $BINARY_DIR not found.\e[0m"
    exit 1
fi
if [ ! -d "$SCRIPTS_DIR" ]; then
    echo -e "\e[31mError: scripts directory $SCRIPTS_DIR not found.\e[0m"
    exit 1
fi
if [ ! -d "$RESOURCES_MEDIAMTX_DIR" ]; then
    echo -e "\e[31mError: mediamtx directory $RESOURCES_MEDIAMTX_DIR not found.\e[0m"
    exit 1
fi

echo "Identifying binaries..."
IFS=$'\n' BINARIES=($(find "$BINARY_DIR" -maxdepth 1 \( -type f -o -type l \)))

BINARIES_TO_DEPLOY=()
SCRIPTS_TO_DEPLOY=()
RESOURCES_TO_DEPLOY=()
ROS2_SRC_TO_DEPLOY=()
CONFIGS_TO_DEPLOY=()

if [ ${#SELECTED_BINARIES[@]} -eq 0 ]; then
    BINARIES_TO_DEPLOY=("${BINARIES[@]}")
    SCRIPT_FILES=("RBQ.bash" "start_motion.bash" "start_vision.bash" "start_mediamtx.bash" "start_vpn.bash" "start_ros_driver.bash" "start_slam.bash")
    for SCRIPT in "${SCRIPT_FILES[@]}"; do
        if [ -f "$SCRIPTS_DIR/$SCRIPT" ]; then
            SCRIPTS_TO_DEPLOY+=("$SCRIPTS_DIR/$SCRIPT")
            echo "  - Added $SCRIPT"
        else
            echo -e "\e[33mWarning: $SCRIPT not found in $SCRIPTS_DIR, skipping.\e[0m"
        fi
    done
    
    if [ -d "$ROS2_SRC_DIR" ]; then
        ROS2_SRC_TO_DEPLOY+=("$ROS2_SRC_DIR")
        echo "  - Added ros2/src for deployment"
    else
        echo -e "\e[33mWarning: $ROS2_SRC_DIR directory not found, skipping.\e[0m"
    fi
    
    RESOURCES_TO_DEPLOY+=("$RESOURCES_MEDIAMTX_DIR")

    if [ ${#BINARIES_TO_DEPLOY[@]} -eq 0 ] && [ ${#SCRIPTS_TO_DEPLOY[@]} -eq 0 ] && [ ${#RESOURCES_TO_DEPLOY[@]} -eq 0 ] && [ ${#ROS2_SRC_TO_DEPLOY[@]} -eq 0 ]; then
        echo -e "\e[31mError: No files to deploy.\e[0m"
        exit 1
    fi
    CONFIGS_TO_DEPLOY+=("configs/cyclonedds_ros2.xml")
    echo "  - Added configs/cyclonedds_ros2.xml for deployment"
else
    for FILE in "${SELECTED_BINARIES[@]}"; do
        if [ -f "$BINARY_DIR/$FILE" ]; then
            BINARIES_TO_DEPLOY+=("$BINARY_DIR/$FILE")
        else
            echo -e "\e[33mWarning: $FILE not found, skipping.\e[0m"
        fi
    done
    if [ ${#BINARIES_TO_DEPLOY[@]} -eq 0 ]; then
        echo -e "\e[31mError: No files to deploy.\e[0m"
        exit 1
    fi
fi

echo -e "\e[33mEnter SSH password.\e[0m"
read -s -p "SSH Password: " SSH_PASSWORD
echo ""
SSH_AUTH="sshpass -p $SSH_PASSWORD ssh"

if [ ! -f "$HOME/.ssh/known_hosts" ]; then
    echo -e "\e[33mNo ~/.ssh/known_hosts yet. Now will ssh to the robot. Type the password then type exit to return\e[0m"
    ssh $REMOTE_DEVICE
fi
ssh-keyscan -H $(echo "$REMOTE_DEVICE" | cut -d'@' -f2) >> ~/.ssh/known_hosts 2>/dev/null

if [ ${#BINARIES_TO_DEPLOY[@]} -gt 0 ]; then
    echo "File transfer starting..."
    if ! rsync -avz --progress --rsync-path="mkdir -p ${REMOTE_BIN_DIR[@]} && rsync" -e "$SSH_AUTH" "${BINARIES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_BIN_DIR/"; then
        echo -e "\e[31mError: File transfer failed.\e[0m"
        exit 1
    fi
fi
if ! rsync -avz --progress -e "$SSH_AUTH" "${SCRIPTS_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_SCRIPTS_DIR"; then
    echo -e "\e[31mError: Script file transfer failed.\e[0m"
    exit 1
fi
if [ ${#ROS2_SRC_TO_DEPLOY[@]} -gt 0 ]; then
    echo "Deploying ros2/src to $REMOTE_ROS2_SRC_DIR..."
    if ! rsync -avz --progress --rsync-path="mkdir -p ${REMOTE_ROS2_SRC_DIR[@]} && rsync" -e "$SSH_AUTH" "${ROS2_SRC_TO_DEPLOY[@]}/" "$REMOTE_DEVICE:$REMOTE_ROS2_SRC_DIR/"; then
        echo -e "\e[31mError: ros2/src file transfer failed.\e[0m"
        exit 1
    fi
fi
if ! rsync -avz --progress -e "$SSH_AUTH" "${RESOURCES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_RESOURCES_DIR/"; then
    echo -e "\e[31mError: Resources/mediamtx file transfer failed.\e[0m"
    exit 1
fi
if [ ${#CONFIGS_TO_DEPLOY[@]} -gt 0 ]; then
    echo "Deploying ${CONFIGS_TO_DEPLOY[@]} to $REMOTE_DEVICE:$REMOTE_CONFIGS_DIR..."
    if ! rsync -avz --progress --rsync-path="mkdir -p ${REMOTE_CONFIGS_DIR[@]} && rsync" -e "$SSH_AUTH" "${CONFIGS_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_CONFIGS_DIR/"; then
        echo -e "\e[31mError: ${CONFIGS_TO_DEPLOY[@]} file transfer failed.\e[0m"
        exit 1
    fi
fi

echo -e "\e[32mDeployment complete.\e[0m"

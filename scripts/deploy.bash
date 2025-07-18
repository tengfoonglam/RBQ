#!/bin/bash

REMOTE_DEVICE="rbq@192.168.0.10"
REMOTE_DIR="~/rbq_ws/bin"
BINARY_DIR="bin"
LOGFILE="deploy.log"

SELECTED_FILES=()

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
        --pro) shift; while [[ $# -gt 0 && ! $1 =~ ^- ]]; do SELECTED_FILES+=("$1"); shift; done ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

echo "Identifying binaries..."
IFS=$'\n' BINARIES=($(find "$BINARY_DIR" -maxdepth 1 \( -type f -o -type l \)))
FILES_TO_DEPLOY=()

if [ ${#SELECTED_FILES[@]} -eq 0 ]; then
    FILES_TO_DEPLOY=("${BINARIES[@]}")
else
    for FILE in "${SELECTED_FILES[@]}"; do
        if [ -f "$BINARY_DIR/$FILE" ]; then
            FILES_TO_DEPLOY+=("$BINARY_DIR/$FILE")
        else
            echo -e "\e[33mWarning: $FILE not found, skipping.\e[0m"
        fi
    done
fi

if [ ${#FILES_TO_DEPLOY[@]} -eq 0 ]; then
    echo -e "\e[31mError: No files to deploy.\e[0m"
    exit 1
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

$SSH_AUTH "$REMOTE_DEVICE" "mkdir -p '$REMOTE_DIR'"

echo "Starting file upload with rsync..."
if ! rsync -avz --progress -e "$SSH_AUTH" "${FILES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_DIR"; then
    echo -e "\e[31mError: File transfer failed.\e[0m"
    exit 1
fi

echo -e "\e[32mDeployment complete.\e[0m"

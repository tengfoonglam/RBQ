#!/bin/bash

APP_NAME="Vision"
APP_PATH="bin"
CMD_ARGS=()
SUDO_MODE=false

print_help() {
    echo "Usage: bash scripts/start_vision.bash [OPTIONS]"
    echo "Options:"
    echo "  --help      Display this help message and exit."
    echo "  --sim       Run in simulator mode."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) SUDO_MODE=true; CMD_ARGS+=("--sim"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

echo -ne "\033]0;$APP_NAME\007"
# if [ "$EUID" -eq 0 ]; then
#     echo "Do not run this script with sudo. Exiting..."
#     exit 1
# fi
if pgrep -x "$APP_NAME" > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi
if [ ! -f "$APP_PATH/$APP_NAME" ]; then
    echo "$APP_NAME application not found."
    echo "Compile it first with: bash scripts/docker/run.bash"
    sleep 10
    exit 1
fi
cd $APP_PATH
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        if [ "$SUDO_MODE" = true ]; then
            sudo ./"$APP_NAME" "${CMD_ARGS[@]}"
        else
            ./"$APP_NAME" "${CMD_ARGS[@]}"
        fi
    fi
    sleep 2
done

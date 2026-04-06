#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

#sudo bash scripts/install/apt.bash
bash scripts/install/cmake.bash
bash scripts/install/json.bash
bash scripts/install/onnx.bash

#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

sudo bash scripts/ex/install/apt.bash
sudo bash scripts/ex/install/ros.bash

bash scripts/ex/install/cmake.bash
bash scripts/ex/install/eigen.bash
bash scripts/ex/install/dds.bash
bash scripts/ex/install/onnx.bash
bash scripts/ex/install/json.bash

#!/usr/bin/env bash

set -e

# Function to retry apt update
try_apt_update() {
    for i in {1..5}; do
        echo "Running apt update... attempt $i"
        apt-get update --quiet && return 0
        sleep 2
    done
    echo "❌ apt update failed after multiple retries." >&2
    exit 1
}

# Clean apt cache and reset
apt-get clean
rm -rf /var/lib/apt/lists/*
try_apt_update
apt-get upgrade --yes --quiet
dpkg --configure -a

# ROS 2 configuration
ROS2_VERSION="humble"
ROS2_DIR="/opt/ros/$ROS2_VERSION"
ROS2_SETUP="$ROS2_DIR/setup.bash"

if [ ! -d "$ROS2_DIR" ]; then
    echo "🔧 Installing ROS 2 $ROS2_VERSION..."

    # Locale setup
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add universe repo
    add-apt-repository universe

    # Setup ROS 2 apt repo and key
    try_apt_update
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    try_apt_update
    apt-get upgrade --yes --quiet

    # Install base ROS 2 packages
    DEBIAN_FRONTEND=noninteractive apt-get install --yes --quiet --no-install-recommends \
        ros-$ROS2_VERSION-desktop \
        ros-$ROS2_VERSION-pcl-ros \
        ros-$ROS2_VERSION-xacro \
        python3-colcon-common-extensions \
        python3-rospkg \
        python3-rosdep \
        ros-$ROS2_VERSION-diagnostic-updater

    # ROS environment setup
    if [ -f "$ROS2_SETUP" ]; then
        echo "source $ROS2_SETUP" >> /etc/profile
        echo "source $ROS2_SETUP" >> ~/.bashrc
        source "$ROS2_SETUP"
    else
        echo "❌ ROS 2 setup file not found at $ROS2_SETUP. Installation may have failed." >&2
        exit 1
    fi

    # rosdep initialization
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        rosdep init
    fi
    rosdep update
fi

echo "✅ ROS 2 version $ROS2_VERSION is installed at $ROS2_DIR"

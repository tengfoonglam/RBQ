#!/usr/bin/env bash
set -e

echo "🔧 Starting SLAM component setup..."

# ========================
# 1. Livox-SDK2 설치
# ========================
echo "🚀 Installing Livox-SDK2..."
sdk_dir="$(realpath "$script_dir/../../../3rdparty/Livox-SDK2")"
if [ ! -d "$sdk_dir" ]; then
    git clone --branch v1.2.5 https://github.com/Livox-SDK/Livox-SDK2.git "$sdk_dir"
    mkdir -p "$sdk_dir/build"
    cd "$sdk_dir/build"
    cmake .. -DCMAKE_INSTALL_PREFIX="$sdk_dir/install"
    make -j$(nproc)
    make install  # sudo 제거
    cd "$script_dir"  # 원래 위치로 복귀
fi

echo "✅ Livox-SDK2 installed."


# ========================
# 2. livox_ros_driver2 설치 및 빌드
# ========================
echo "🚀 Installing livox_ros_driver2..."
source /opt/ros/humble/setup.bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="$(realpath "$script_dir/../..")"
src_dir="$ws_dir/src"
pkg_dir="$script_dir/livox_ros_driver2"

if [ ! -d "$pkg_dir" ]; then
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$pkg_dir"
fi

cp "$script_dir/configs/livox_ros_driver2/MID360_config.json" "$pkg_dir/config/"
cd "$pkg_dir"
rm -f package.xml && cp package_ROS2.xml package.xml
rm -rf launch && cp -r launch_ROS2 launch


# ========================
# 3. FAST_LIO 설치 및 의존성
# ========================
echo "🚀 Installing FAST_LIO..."

fastlio_dir="$script_dir/FAST_LIO"
if [ ! -d "$fastlio_dir" ]; then
    git clone https://github.com/Ericsii/FAST_LIO.git --recursive "$fastlio_dir"
fi

cp "$script_dir/configs/fast_lio/rbq_mid360.yaml" "$fastlio_dir/config"

cd "$ws_dir"
rosdep update
rosdep install --from-paths src --ignore-src -y


# ========================
# 4. 전체 colcon build
# ========================
colcon build --base-paths src/rbq_slam_example

# ========================
# 5. .bashrc 등록 및 환경 설정
# ========================
ros_setup_line="source /opt/ros/humble/setup.bash"
ws_setup_line="source ~/rbq_ws/ros2/install/setup.bash"

grep -qxF "$ros_setup_line" ~/.bashrc || echo "$ros_setup_line" >> ~/.bashrc
grep -qxF "$ws_setup_line" ~/.bashrc || echo "$ws_setup_line" >> ~/.bashrc

# 현재 셸에도 적용
source /opt/ros/humble/setup.bash
source ~/rbq_ws/ros2/install/setup.bash

echo "✅ SLAM setup completed successfully."
echo "ℹ️  Please run 'source ~/.bashrc' or reopen the terminal to apply changes."


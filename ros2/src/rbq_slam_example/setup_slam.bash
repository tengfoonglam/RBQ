#!/usr/bin/env bash
set -e

echo "🔧 Starting SLAM component setup..."


slam_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" #/rbq_ws/RBQ/jw/ros2/src/rbq_slam_example
ros2_dir="$(realpath "$slam_dir/../..")" #ros2    
src_dir="$ros2_dir/src" #ros2/src
livox_ros_dir="$slam_dir/livox_ros_driver2" 
fastlio_dir="$slam_dir/FAST_LIO"

# ========================
# 1. Livox-SDK2 설치
# ========================
echo "🚀 Installing Livox-SDK2..."
if [ ! -d "Livox-SDK2" ]; then
    git clone --branch v1.2.5 https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    cd ../../  # root로 복귀
fi
echo "✅ Livox-SDK2 installed."


# ========================
# 2. livox_ros_driver2 설치 및 빌드
# ========================
echo "🚀 Installing livox_ros_driver2..."
source /opt/ros/humble/setup.bash


if [ ! -d "$livox_ros_dir" ]; then
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$livox_ros_dir"
fi

cp "$slam_dir/configs/livox_ros_driver2/MID360_config.json" "$livox_ros_dir/config/"
cd "$livox_ros_dir"
rm -f package.xml && cp package_ROS2.xml package.xml
rm -rf launch && cp -r launch_ROS2 launch

# build.sh
pushd `pwd` > /dev/null
cd `dirname $0`

rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
rm -f package.xml
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

cd "$ros2_dir"
colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

cd "$livox_ros_dir"
rm -rf launch/
popd > /dev/null

echo "✅livox_ros_driver2 Build completed!" 

# ========================
# 3. FAST_LIO 설치 및 의존성
# ========================
echo "🚀 Installing FAST_LIO..."

if [ ! -d "$fastlio_dir" ]; then
    git clone https://github.com/Ericsii/FAST_LIO.git --recursive "$fastlio_dir"
fi

cp "$slam_dir/configs/fast_lio/rbq_mid360.yaml" "$fastlio_dir/config"
cd "$ros2_dir"
rosdep update
rosdep install --from-paths src --ignore-src -y


# ========================
# 4. 전체 colcon build
# ========================
cd "$livox_ros_dir"

# build.sh
pushd `pwd` > /dev/null
cd `dirname $0`

rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
rm -f package.xml
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

cd "$ros2_dir"
colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

cd "$livox_ros_dir"
rm -rf launch/

popd > /dev/null

echo "✅All Build completed!" 


# ========================
# 5. .bashrc 등록 및 환경 설정
# ========================
ros_setup_line="source /opt/ros/humble/setup.bash"
ws_setup_line="source $ros2_dir/install/setup.bash"

grep -qxF "$ros_setup_line" ~/.bashrc || echo "$ros_setup_line" >> ~/.bashrc
grep -qxF "$ws_setup_line" ~/.bashrc || echo "$ws_setup_line" >> ~/.bashrc

# 현재 셸에도 적용
source /opt/ros/humble/setup.bash
source "$ros2_dir"/install/setup.bash

echo "✅ SLAM setup completed successfully."
echo "ℹ️  Please run 'source ~/.bashrc' or reopen the terminal to apply changes."

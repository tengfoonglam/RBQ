#!/bin/bash

USE_NINJA=true
MAKE_JOBS=$(nproc)
USE_CACHE=true
CURRENT_DIR="$PWD"
EXAMPLES_DIR="examples"
BUILD_DIR="build"
BIN_DIR="bin"
RCL_DIR="$PWD/bin/rcl"
ROS_DIR="$PWD/ros2"
RBDL_DIR="$PWD/3rdparty/rbdl"
BUILD_ROS=false

print_help() {
    echo "Usage: bash scripts/example_build.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  -j [number]             Specify number of CPUs for parallel build."
    echo "  --use-make              Use Make instead of Ninja."
    echo "  --ros                   Build ros2 applications."
}

# === Argument Parsing ===
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache) USE_CACHE=false;    shift ;;
        -j) shift;  MAKE_JOBS="${1:-$(nproc)}"; shift ;;
        --use-make) USE_NINJA=false;    shift ;;
        --ros)      BUILD_ROS=true;     shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ ! -f scripts/configure.bash ]; then
    echo "[ERROR] scripts/configure.bash not exist!"
    exit 1
fi
source scripts/configure.bash

if [ ! -d $RCL_DIR ]; then
    echo "[ERROR] $RCL_DIR directory not exist!"
    exit 1
fi

if [ ! -d $EXAMPLES_DIR ]; then
    echo "[ERROR] $EXAMPLES_DIR directory not exist!"
    exit 1
fi
cd $EXAMPLES_DIR

if ! $USE_CACHE; then
    echo "[INFO] Cache bypassed. Cleaning build and bin directories..."
    rm -rf "$BUILD_DIR" "$BIN_DIR"
fi
if ! $USE_CACHE || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    GENERATOR="Ninja"
    $USE_NINJA || GENERATOR="Unix Makefiles"

    echo "[INFO] Using generator: $GENERATOR"
    echo "[INFO] Configuring CMake..."

    CMAKE_OPTIONS=()
    CMAKE_OPTIONS+=("-D CMAKE_BUILD_TYPE=Release")
    CMAKE_OPTIONS+=("-D BUILD_SHARED_LIBS=OFF")
    CMAKE_OPTIONS+=("-D CMAKE_INSTALL_PREFIX=$PWD")
    CMAKE_OPTIONS+=("-D CUSTOM_RCL_PATH=$RCL_DIR")
    CMAKE_OPTIONS+=("-D CUSTOM_RBDL_PATH=$RBDL_DIR")
    CMAKE_PREFIX_PATH_STRING="$RCL_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$EIGEN_DIR/share/eigen3/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$DDS_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$DDS_CXX_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$ONNX_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$QT_STATIC_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$RBDL_DIR/lib/cmake"

    CMAKE_OPTIONS+=("-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH_STRING")

    cmake -S . -B "$BUILD_DIR" -G "$GENERATOR" "${CMAKE_OPTIONS[@]}" || {
        echo "[ERROR] CMake configuration failed!"
        exit 1
    }
fi
echo "[INFO] Building the project..."
cmake --build "$BUILD_DIR" -j"$MAKE_JOBS" || {
    echo "[ERROR] CMake build failed!"
    exit 1
}
echo "[INFO] Installing the project..."
cmake --install "$BUILD_DIR" || {
    echo "[ERROR] CMake install failed!"
    exit 1
}
if [ -d $BIN_DIR ]; then
    BINARIES=($(find "$BIN_DIR" -maxdepth 1 -type f -executable))
    if [ ${#BINARIES[@]} -eq 0 ]; then
        echo "[INFO] No binaries to strip."
    else
        echo "[INFO] Stripping binaries..."
        for f in "${BINARIES[@]}"; do
            strip "$f"
        done
    fi
    echo "✅ Build complete."
fi

if [ "$BUILD_ROS" = true ]; then
    cd $CURRENT_DIR
    if [ ! -d $ROS_DIR ]; then
        echo -e "[ERROR] No ros2 directory found in: $ROS_DIR"
        exit 1
    fi
    cd $ROS_DIR
    if [ ! $USE_CACHE ]; then
        echo "[INFO] Cache bypassed. Cleaning build and bin directories..."
        rm -rf build install log
    fi
    rosdep install --from-paths src -y --ignore-src
    echo "[INFO] Building the ros2 projects..."
    colcon build --symlink-install
    echo "✅ Build ROS2 complete."
fi

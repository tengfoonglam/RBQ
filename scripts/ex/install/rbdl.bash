#!/usr/bin/env bash
set -e
source scripts/configure.bash

# RBDL
if [ ! -d $RBDL_DIR ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf $RBDL_VERSION.tar.gz rbdl*
    echo "[INFO] Downloading RBDL $RBDL_VERSION ..."
    wget https://github.com/rbdl/rbdl/archive/refs/tags/$RBDL_VERSION.tar.gz
    tar xf $RBDL_VERSION.tar.gz && cd rbdl*
    sed -i '/set(CMAKE_EXPORT_COMPILE_COMMANDS ON)/i \
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx -mfma") \n\
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx -mfma")' CMakeLists.txt
    mkdir -p build && cd build
    echo "[INFO] Configuring RBDL CMake..."
    cmake -S .. -B . -G Ninja \
        -DRBDL_BUILD_STATIC=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$RBDL_DIR

    echo "[INFO] Building & Installing RBDL ..."
    cmake --build . -j $(nproc) && cmake --install .
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ RBDL version $RBDL_VERSION installed to $RBDL_DIR"

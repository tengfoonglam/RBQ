#!/usr/bin/env bash
set -e
source scripts/configure.bash

if [ ! -d "$ONNX_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf onnxruntime
    echo "[INFO] Cloning onnx version: $ONNX_VERSION ..."
    git clone --recursive https://github.com/microsoft/onnxruntime.git
    cd onnxruntime
    git checkout $ONNX_VERSION && git submodule update --init --recursive

    echo "[INFO] Building ONNX ..."
    ./build.sh \
        --config Release \
        --parallel \
        --disable_ml_ops \
        --allow_running_as_root \
        --cmake_extra_defines \
            onnxruntime_BUILD_SHARED_LIB=ON \
            onnxruntime_ENABLE_PYTHON=OFF \
            onnxruntime_BUILD_UNIT_TESTS=OFF \
            onnxruntime_ENABLE_LTO=ON \
            onnxruntime_ENABLE_OPENMP=ON \
            onnxruntime_ENABLE_EXCEPTIONS=ON \
            CMAKE_POSITION_INDEPENDENT_CODE=ON \
            CMAKE_INSTALL_PREFIX=$ONNX_DIR \
            onnxruntime_USE_PREINSTALLED_EIGEN=ON \
            eigen_SOURCE_PATH=$EIGEN_DIR/include/eigen3

    echo "[INFO] Installing ONNX to $ONNX_DIR..."
    cd build/Linux/Release && cmake --install .

    echo "[INFO] Stripping binaries..."
    find $ONNX_DIR -type f -executable -exec strip --strip-unneeded {} +

    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd $LIBS_DIR && rm -rf $TMP_DIR
    fi
fi
echo "✅ onnx version $ONNX_VERSION installed to $ONNX_DIR"

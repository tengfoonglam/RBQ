#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

sudo bash scripts/apt.bash

sudo bash scripts/ros.bash

source scripts/configure.bash

# CMAKE
if [ ! -d $CMAKE_DIR ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf cmake-$CMAKE_VERSION-linux-x86_64.tar.gz cmake-$CMAKE_VERSION-linux-x86_64
    echo "[INFO] Downloading cmake $CMAKE_VERSION..."
    wget -q https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/cmake-$CMAKE_VERSION-linux-x86_64.tar.gz
    echo "[INFO] Extracting cmake ..."
    tar -xzf cmake-$CMAKE_VERSION-linux-x86_64.tar.gz
    mkdir -p $CMAKE_DIR && mv cmake-$CMAKE_VERSION-linux-x86_64/* $CMAKE_DIR
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
	cd $LIBS_DIR && rm -rf $TMP_DIR
    fi
fi
echo "✅ cmake version $CMAKE_VERSION installed to $CMAKE_DIR"

# EIGEN
if [ ! -d "$EIGEN_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf eigen-$EIGEN_VERSION.tar.gz eigen-$EIGEN_VERSION
    echo "[INFO] Cloning eigen version: $EIGEN_VERSION ..."
    wget https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VERSION/eigen-$EIGEN_VERSION.tar.gz
    tar -xf eigen-$EIGEN_VERSION.tar.gz && cd eigen-$EIGEN_VERSION
    mkdir build && cd build
    echo "[INFO] Configuring eigen ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$EIGEN_DIR \
        -DEIGEN_LEAVE_TEST_IN_ALL=OFF \
        -DBUILD_TESTING=OFF \
        -DEIGEN_BUILD_DOC=OFF \
        -DEIGEN_BUILD_PKGCONFIG=OFF

    echo "[INFO] Building & Installing eigen to $EIGEN_DIR..."
    cmake --build . -j $(nproc) && cmake --install .
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ eigen version $EIGEN_VERSION installed to $EIGEN_DIR"

# JSON
if [ ! -d "$JSON_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf v$JSON_VERSION.tar.gz json-$JSON_VERSION
    echo "[INFO] Cloning json version: $JSON_VERSION ..."
    wget https://github.com/nlohmann/json/archive/refs/tags/v$JSON_VERSION.tar.gz
    tar -xf v$JSON_VERSION.tar.gz && cd json-$JSON_VERSION
    mkdir build && cd build
    echo "[INFO] Configuring json ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$JSON_DIR \
        -DBUILD_SHARED_LIBS=OFF \
        -DJSON_BuildTests=OFF \
        -DJSON_MultipleHeaders=OFF


    echo "[INFO] Building & Installing json to $JSON_DIR..."
    cmake --build . -j $(nproc) && cmake --install .
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ json version $JSON_VERSION installed to $JSON_DIR"

# ONNX
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

# DDS
if [ ! -d "$DDS_DIR" ]; then
    mkdir -p "$TMP_DIR/dds" && cd "$TMP_DIR/dds"
    rm -rf "$DDS_VERSION.tar.gz" "cyclonedds-$DDS_VERSION"
    echo "[INFO] Downloading dds $DDS_VERSION..."
    wget https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/$DDS_VERSION.tar.gz
    tar -xf $DDS_VERSION.tar.gz && cd cyclonedds-$DDS_VERSION
    mkdir build && cd build
    echo "[INFO] Configuring dds ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$DDS_DIR \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_RPATH="$PWD/lib" \
        -DENABLE_SHM=OFF \
        -DBUILD_DDSPERF=OFF
    echo "[INFO] Building & Installing dds to $DDS_DIR ..."
    cmake --build . -j $(nproc) && cmake --install .
    echo "✅ dds version $DDS_VERSION installed to $DDS_DIR"

    export CMAKE_PREFIX_PATH=$DDS_DIR:$CMAKE_PREFIX_PATH
    export LD_LIBRARY_PATH=$DDS_DIR/lib:$LD_LIBRARY_PATH

    cd "$TMP_DIR/dds"
    rm -rf "$DDS_CXX_VERSION.tar.gz" "cyclonedds-cxx-$DDS_CXX_VERSION"
    wget https://github.com/eclipse-cyclonedds/cyclonedds-cxx/archive/refs/tags/$DDS_CXX_VERSION.tar.gz
    tar -xf $DDS_CXX_VERSION.tar.gz && cd cyclonedds-cxx-$DDS_CXX_VERSION
    mkdir build && cd build
    echo "[INFO] Configuring dds-cxx ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$DDS_CXX_DIR \
        -DCycloneDDS_DIR=$DDS_DIR/share/CycloneDDS/cmake \
        -DCycloneDDS_SOURCE_DIR=$TMP_DIR/dds/cyclonedds-$DDS_VERSION \
        -DENABLE_SHM=OFF
    echo "[INFO] Building & Installing dds-cxx to $DDS_CXX_DIR..."
    cmake --build . -j $(nproc) && cmake --install .
    echo "✅ dds-cxx version $DDS_CXX_VERSION installed to $DDS_CXX_DIR"

    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR"  && rm -rf "$TMP_DIR"
    fi
fi
echo "[INFO] Verifying DDS install..."
"$DDS_DIR/bin/idlc" -v || echo "Warning: idlc version check failed"
test -f "$DDS_DIR/lib/libddsc.so" && echo "✅ libddsc built"
test -f "$DDS_CXX_DIR/lib/libddscxx.so" && echo "✅ libddscxx built"

# ASSIMP
if [ ! -d "$ASSIMP_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf assimp
    echo "[INFO] Cloning assimp version: $ASSIMP_VERSION ..."
        git clone --branch v$ASSIMP_VERSION https://github.com/assimp/assimp.git
    mkdir -p assimp/build && cd assimp/build
    echo "[INFO] Configuring json ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$ASSIMP_DIR \

    echo "[INFO] Building & Installing assimp to $ASSIMP_DIR..."
    cmake --build . -j $(nproc) && cmake --install .
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ assimp installed to $ASSIMP_DIR"

# Qt source download
if [ ! -f "$QT_DIR/$QT_TAR" ]; then
    echo "[INFO] Downloading Qt version $QT_VERSION to $QT_DIR..."
    mkdir -p "$QT_DIR" && cd $QT_DIR
    wget https://download.qt.io/archive/qt/5.15/$QT_VERSION/single/$QT_TAR
    if [ ! -f "$QT_DIR/$QT_TAR" ]; then
        echo "[ERROR] Qt source not found in $QT_DIR/$QT_TAR download failed."
        exit 1
    fi
fi
echo "✅ Qt source $QT_TAR downloaded to: $QT_DIR"

# Qt static build
if [ ! -f "$QT_STATIC_DIR/bin/qmake" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf qt-everywhere-src-$QT_VERSION
    cp -r $QT_DIR/$QT_TAR . && tar xf $QT_TAR && cd qt-everywhere-src-$QT_VERSION

    echo "[INFO] Configuring Qt static $QT_VERSION to $QT_STATIC_DIR..."
    ./configure \
        -prefix "$QT_STATIC_DIR" \
        -release \
        -opensource -confirm-license \
        -silent -nomake examples -nomake tests \
        -qpa xcb \
        -qt-doubleconversion -qt-pcre -qt-zlib -qt-harfbuzz \
        -qt-libpng -qt-libjpeg -qt-tiff \
        -fontconfig \
        -optimize-size -strip -skip qttools \
        -static -no-shared \

    echo "[INFO] Building Qt static ..."
    make -j$(nproc) 2>&1 | tee build.log
    if grep -i "error:" build.log; then
        echo "[ERROR] Build failed due to errors! check build.log for more info"
        exit 1
    fi
    echo "[INFO] Installing Qt static..."
    make install
    if [ ! -f "$QT_STATIC_DIR/bin/qmake" ]; then
        echo "[ERROR] qmake not found in $QT_STATIC_DIR/bin! Installation failed."
        exit 1
    fi
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ Qt static version $QT_VERSION installed to $QT_STATIC_DIR"

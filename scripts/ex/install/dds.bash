#!/usr/bin/env bash
set -e
source scripts/configure.bash

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

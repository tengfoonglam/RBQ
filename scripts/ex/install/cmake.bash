#!/usr/bin/env bash
set -e
source scripts/configure.bash

case "$(uname -m)" in
    x86_64)   ARCH_TAG="x86_64" ;;
    aarch64)  ARCH_TAG="aarch64";;
    armv7l)   ARCH_TAG="armhf"  ;;
    i*86)     ARCH_TAG="i386"   ;;
    *) echo "Unsupported architecture: $(uname -m)" && exit 1 ;;
esac

if [ ! -d $CMAKE_DIR ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf cmake-$CMAKE_VERSION-linux-${ARCH_TAG}.tar.gz cmake-$CMAKE_VERSION-linux-${ARCH_TAG}
    echo "[INFO] Downloading cmake $CMAKE_VERSION..."
    wget -q https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/cmake-$CMAKE_VERSION-linux-${ARCH_TAG}.tar.gz
    echo "[INFO] Extracting cmake ..."
    tar -xzf cmake-$CMAKE_VERSION-linux-${ARCH_TAG}.tar.gz
    mkdir -p $CMAKE_DIR && mv cmake-$CMAKE_VERSION-linux-${ARCH_TAG}/* $CMAKE_DIR
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
	cd $LIBS_DIR && rm -rf $TMP_DIR
    fi
fi
echo "✅ cmake version $CMAKE_VERSION installed to $CMAKE_DIR"

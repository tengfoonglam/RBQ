#!/usr/bin/env bash
set -e
source scripts/configure.bash

if [ ! -f "$QT_STATIC_DIR/bin/qmake" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf qt-everywhere-src-$QT_VERSION
    cp -r $QT_DIR/$QT_TAR . && tar xf $QT_TAR && cd qt-everywhere-src-$QT_VERSION

    perl -0777 -i.bak -pe 's/^\s*(QMAKE_USE_PRIVATE\s*\+=\s*)assimp\b/${1}quick3d-assimp/m' qtquick3d/src/plugins/assetimporters/assimp/assimp.pro

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

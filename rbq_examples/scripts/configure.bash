#!/usr/bin/env bash
set -e

export REMOVE_TMP=true

export LIBS_DIR="$PWD/3rdparty"
export TMP_DIR="$LIBS_DIR/tmp"

# cmake
export CMAKE_VERSION="3.27.9"
export CMAKE_DIR="$LIBS_DIR/cmake"
export PATH=$CMAKE_DIR/bin:$PATH

# json
export JSON_VERSION="3.12.0"
export JSON_DIR="$LIBS_DIR/json"

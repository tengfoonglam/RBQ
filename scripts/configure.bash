#!/usr/bin/env bash
set -e

export REMOVE_TMP=true

export LIBS_DIR="$PWD/3rdparty"
export TMP_DIR="$LIBS_DIR/tmp"

# cmake
export CMAKE_VERSION="3.27.9"
export CMAKE_DIR="$LIBS_DIR/cmake"
export PATH=$CMAKE_DIR/bin:$PATH

# eigen
export EIGEN_VERSION="3.4.0"
export EIGEN_DIR="$LIBS_DIR/eigen"

# json
export JSON_VERSION="3.12.0"
export JSON_DIR="$LIBS_DIR/json"

# ONNX
export ONNX_VERSION="v1.17.1"
export ONNX_DIR="$LIBS_DIR/onnx"

# RBDL
export RBDL_VERSION="v3.3.1"
export RBDL_DIR="$LIBS_DIR/rbdl"

# dds
export DDS_VERSION="0.10.5"
export DDS_DIR="$LIBS_DIR/dds"
export DDS_CXX_VERSION="0.10.5"
export DDS_CXX_DIR="$LIBS_DIR/ddscxx"
export PATH=$DDS_DIR/bin:$PATH
export LD_LIBRARY_PATH=$DDS_CXX_DIR/lib:$LD_LIBRARY_PATH

# RCL
export RCL_DIR="$PWD/bin/rcl"

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:99

WORKDIR /workspace

ENV SCRIPTS_DIR="scripts/ex"
RUN mkdir -p $SCRIPTS_DIR

# APT
COPY $SCRIPTS_DIR/install/apt.bash $SCRIPTS_DIR/apt.bash
RUN bash $SCRIPTS_DIR/apt.bash
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales

# Configure
COPY $SCRIPTS_DIR/configure.bash $SCRIPTS_DIR/../configure.bash

# cmake
COPY $SCRIPTS_DIR/install/cmake.bash $SCRIPTS_DIR/cmake.bash
RUN bash $SCRIPTS_DIR/cmake.bash

# Eigen
COPY $SCRIPTS_DIR/install/eigen.bash $SCRIPTS_DIR/eigen.bash
RUN bash $SCRIPTS_DIR/eigen.bash

# DDS
COPY $SCRIPTS_DIR/install/dds.bash $SCRIPTS_DIR/dds.bash
RUN bash $SCRIPTS_DIR/dds.bash

# ONNX
COPY $SCRIPTS_DIR/install/onnx.bash $SCRIPTS_DIR/onnx.bash
RUN bash $SCRIPTS_DIR/onnx.bash

# RBDL
COPY $SCRIPTS_DIR/install/rbdl.bash $SCRIPTS_DIR/rbdl.bash
RUN bash $SCRIPTS_DIR/rbdl.bash

# JSON
COPY $SCRIPTS_DIR/install/json.bash $SCRIPTS_DIR/json.bash
RUN bash $SCRIPTS_DIR/json.bash

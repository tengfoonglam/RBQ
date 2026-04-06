FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:99

WORKDIR /workspace

ENV SCRIPTS_DIR="/workspace/scripts"
RUN mkdir -p $SCRIPTS_DIR

# Build context must be the repo root (parent of rbq_examples) so rbq_sdk
# files are included. See scripts/docker/run.bash.

# APT
COPY rbq_examples/scripts/install/apt.bash $SCRIPTS_DIR/apt.bash
RUN bash $SCRIPTS_DIR/apt.bash
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales

# Configure
COPY rbq_examples/scripts/configure.bash $SCRIPTS_DIR/configure.bash

# cmake
COPY rbq_examples/scripts/install/cmake.bash $SCRIPTS_DIR/cmake.bash
RUN bash $SCRIPTS_DIR/cmake.bash

# ONNX
COPY rbq_examples/scripts/install/onnx.bash $SCRIPTS_DIR/onnx.bash
RUN bash $SCRIPTS_DIR/onnx.bash

# JSON
COPY rbq_examples/scripts/install/json.bash $SCRIPTS_DIR/json.bash
RUN bash $SCRIPTS_DIR/json.bash

# DDS
COPY rbq_sdk/scripts/build_dds.bash $SCRIPTS_DIR/build_dds.bash
RUN bash $SCRIPTS_DIR/build_dds.bash

# RBQ SDK
COPY rbq_sdk/ /workspace/rbq_sdk/
RUN cd /workspace/rbq_sdk && bash scripts/build.bash --no-cache
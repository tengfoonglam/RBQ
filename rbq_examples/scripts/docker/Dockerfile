FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:99
ENV NO_AT_BRIDGE=1

WORKDIR /workspace

# Build context must be the repo root (parent of rbq_examples) so rbq_sdk
# files are included. See scripts/docker/run.bash.

# APT
COPY rbq_examples/scripts/debian-dep.bash scripts/debian-dep.bash
RUN bash scripts/debian-dep.bash
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales

# DDS
COPY rbq_sdk/scripts/build_dds.bash scripts/build_dds.bash
RUN bash scripts/build_dds.bash

# RBQ SDK
COPY rbq_sdk/ /workspace/rbq_sdk/
RUN cd /workspace/rbq_sdk && bash scripts/build.bash --no-cache

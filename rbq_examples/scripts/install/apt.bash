#!/usr/bin/env bash
set -e

apt clean
rm -rf /var/lib/apt/lists/*
apt update --quiet -o Acquire::BrokenProxy=true -o Acquire::AllowInsecureRepositories=true || true
apt upgrade -y --quiet

dpkg --configure -a

DEBIAN_FRONTEND=noninteractive apt -y --quiet --no-install-recommends install  \
    locales ca-certificates apt-transport-https \
    cmake build-essential pkg-config ninja-build \
    git curl wget sshpass libssl-dev libcurl4-openssl-dev \
    python3 python3-dev python3-pip \
    libeigen3-dev \

echo "✅ apt installed."

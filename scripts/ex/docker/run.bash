#!/usr/bin/env bash
set -e

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Dynamically set Docker image name based on Git branch
RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
IMAGE_NAME="rbq-ex-${SANITIZED_BRANCH_NAME}"
NO_CHECK=false
CMD_ARGS=()
DIR="examples"

print_help() {
    echo "Usage: bash scripts/ex/docker/run.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-check              Bypass docker image check."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  --use-make              Use Make instead of Ninja."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)         print_help; exit 0 ;;
        --no-check)     NO_CHECK=true; shift ;;
        --no-cache)     CMD_ARGS+=("--no-cache"); shift ;;
        --use-make)     CMD_ARGS+=("--use-make"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ ! -d $DIR ]; then
    echo "[ERROR] $DIR directory not found, run this script from the root of the repository."
    exit 1
fi
cleanup_on_failure() {
    if [[ $? -ne 0 ]]; then
        echo "[ERROR] Build failed. Cleaning up cache..."
        sudo chown -R $(logname):$(logname) $DIR
        rm -rf $DIR/build $DIR/bin
    fi
}
trap cleanup_on_failure EXIT
if [[ "$NO_CHECK" == "false" ]]; then
    sudo snap install docker
    echo "Docker container build starting..."
    sudo docker build \
        --network host \
        --file scripts/ex/docker/Dockerfile \
        -t $IMAGE_NAME .
fi
CMD="bash build.bash ${CMD_ARGS[@]}"
sudo docker run \
    --rm \
    --cap-add SYS_ADMIN \
    --device /dev/fuse \
    --security-opt apparmor:unconfined \
    --network host \
    -v ${PWD}/bin:/workspace/bin \
    -v ${PWD}/$DIR:/workspace/$DIR \
    -v ${PWD}/scripts/ex/build.bash:/workspace/build.bash \
    $IMAGE_NAME bash -c "$CMD"

trap - EXIT
echo "Docker container executed successfully."
sudo chown -R $(logname):$(logname) $DIR

#!/bin/bash

# Cross-platform Docker build script for ROS 2 + Gazebo
# Supports: Linux (x86_64, ARM64), macOS (Intel, Apple Silicon)

set -e

# Get the absolute path of the directory containing this script (scripts directory)
SCRIPT_PATH=$(dirname $(realpath "$0"))

# Get the devcontainer directory path
DEVCONTAINER_PATH=$(dirname "$SCRIPT_PATH")

# Get the workspace root directory (ros2_jazzy_ws)
WORKSPACE_ROOT=$(dirname "$DEVCONTAINER_PATH")

# Image configuration
IMAGE_NAME="manipulation"
IMAGE_TAG="latest"

# Detect host OS and architecture
detect_platform() {
    HOST_OS=$(uname -s)
    HOST_ARCH=$(uname -m)
    
    echo ""
    echo "Detected: $HOST_OS ($HOST_ARCH)"
    
    # ROS Jazzy desktop image is only available for linux/amd64
    # We must build with amd64 platform - ARM Macs will use Rosetta 2 emulation
    BUILD_PLATFORM="linux/amd64"
    
    if [[ "$HOST_OS" == "Darwin" ]]; then
        echo "Platform: macOS - will use Docker Desktop with Rosetta 2 emulation"
        if [[ "$HOST_ARCH" == "arm64" ]]; then
            echo "Note: Apple Silicon detected - x86_64 emulation will be used"
        fi
    else
        echo "Platform: Linux ($HOST_ARCH)"
    fi
    echo "Build target: $BUILD_PLATFORM"
    echo ""
}

# Function to print debug messages
print_debug() {
    echo ""
    echo "$LOG"
    echo ""
}

# Function to create a shared folder
create_shared_folder() {
    if [ ! -d "$HOME/sahrudaypatti/shared/ros2" ]; then
        LOG="Creating $HOME/sahrudaypatti/shared/ros2 ..."
        print_debug
        mkdir -p "$HOME/sahrudaypatti/shared/ros2"
    fi
}

# Function to build the Docker image
build_docker_image() {
    LOG="Building Docker image ${IMAGE_NAME}:${IMAGE_TAG} ..."
    print_debug

    # Build with explicit platform to avoid warnings and ensure consistent builds
    docker image build \
        --platform "$BUILD_PLATFORM" \
        -f "$DEVCONTAINER_PATH/docker/Dockerfile" \
        -t "${IMAGE_NAME}:${IMAGE_TAG}" \
        "$WORKSPACE_ROOT"
}

# Main execution flow
detect_platform
create_shared_folder
build_docker_image

echo ""
echo "Build complete!"
echo "  Linux: ./.devcontainer/scripts/run.sh"
echo "  macOS: ./.devcontainer/scripts/run-novnc.sh"
echo ""

#!/bin/bash

# Docker run script for ROS 2 + Gazebo simulation
#
# PLATFORM SUPPORT:
#   Linux: Full GUI support via X11 forwarding
#   macOS: Use run-novnc.sh instead for GUI applications (Gazebo, RViz)
#          This script can run headless/CLI operations on macOS
#
# Prerequisites (Linux):
#   xhost +local:docker

set -e

# Get the absolute path of the directory containing this script
SCRIPT_PATH=$(dirname $(realpath "$0"))
DEVCONTAINER_PATH=$(dirname "$SCRIPT_PATH")
WORKSPACE_ROOT=$(dirname "$DEVCONTAINER_PATH")

# Image configuration  
IMAGE_NAME="manipulation"
IMAGE_TAG="latest"

# Detect host OS
HOST_OS=$(uname -s)

echo ""
echo "=== ROS 2 Jazzy + Gazebo Container ==="
echo "Host OS: $HOST_OS"
echo ""

# Build docker run arguments based on host OS
build_docker_args() {
    # Common arguments
    DOCKER_ARGS=(
        "--rm"
        "--platform" "linux/amd64"
        "--cap-add=SYS_PTRACE"
        "--security-opt" "seccomp=unconfined"
    )

    # Add interactive TTY flags only if stdin is a terminal
    if [ -t 0 ]; then
        DOCKER_ARGS+=("-it")
    else
        DOCKER_ARGS+=("-i")
    fi

    # Graphics/rendering environment for macOS XQuartz compatibility
    DOCKER_ARGS+=(
        # Disable X11 shared memory (not supported across Docker/XQuartz)
        "-e" "QT_X11_NO_MITSHM=1"
        "-e" "MESA_NO_SHM=1"
        "-e" "_X11_NO_MITSHM=1"
        
        # Disable GLX - XQuartz only supports OpenGL 2.1
        "-e" "QT_XCB_GL_INTEGRATION=none"
        
        # Force Qt to use software rendering for Quick/QML
        "-e" "QT_QUICK_BACKEND=software"
        "-e" "QTWEBENGINE_CHROMIUM_FLAGS=--disable-gpu"
        
        # EGL configuration for Gazebo's headless rendering
        "-e" "__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json"
        "-e" "EGL_PLATFORM=surfaceless"
        
        # Mesa software rendering
        "-e" "LIBGL_ALWAYS_SOFTWARE=1"
        "-e" "MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
        "-e" "GALLIUM_DRIVER=llvmpipe"
        "-e" "MESA_GL_VERSION_OVERRIDE=3.3"
        "-e" "MESA_GLSL_VERSION_OVERRIDE=330"
        
        # Gazebo rendering settings
        "-e" "GZ_SIM_RENDER_ENGINE=ogre"
        "-e" "OGRE_RTT_MODE=Copy"
    )

    # OS-specific configuration
    case "$HOST_OS" in
        "Linux")
            configure_linux
            ;;
        "Darwin")
            configure_macos
            ;;
        *)
            echo "Unsupported OS: $HOST_OS"
            exit 1
            ;;
    esac

    # Volume mounts (common)
    DOCKER_ARGS+=(
        "-v" "${WORKSPACE_ROOT}/workspace:/root/ros2_ws/src"
        "-v" "${WORKSPACE_ROOT}/results:/root/ros2_ws/src/puma560_ros2_moveit/results"
    )
}

configure_linux() {
    echo "Configuring for Linux X11..."
    
    # Verify X11 is accessible
    if [ -z "$DISPLAY" ]; then
        echo "Warning: DISPLAY not set. GUI applications may not work."
    fi

    # Network mode (host works well on Linux)
    DOCKER_ARGS+=("--network=host")

    # X11 display
    DOCKER_ARGS+=(
        "-e" "DISPLAY=${DISPLAY}"
        "-v" "/tmp/.X11-unix:/tmp/.X11-unix"
    )

    # GPU access if available (optional, falls back to software rendering)
    if [ -d "/dev/dri" ]; then
        DOCKER_ARGS+=("--device=/dev/dri")
        echo "GPU access enabled (/dev/dri)"
    fi

    # USB access if available (optional, for real hardware)
    if [ -d "/dev/bus/usb" ]; then
        DOCKER_ARGS+=("-v" "/dev/bus/usb:/dev/bus/usb")
    fi

    echo "X11 display: $DISPLAY"
    echo ""
    echo "Reminder: Run 'xhost +local:docker' if you haven't already"
}

configure_macos() {
    echo ""
    echo "=================================================="
    echo "  macOS Detected"
    echo "=================================================="
    echo ""
    echo "For GUI applications (Gazebo, RViz), use noVNC instead:"
    echo "  ./.devcontainer/run-novnc.sh"
    echo "  Then open http://localhost:6080/vnc.html"
    echo ""
    echo "This script will start the container for CLI/headless use."
    echo "XQuartz does not support the OpenGL version required by Gazebo."
    echo ""
    echo "=================================================="
    echo ""
    
    # Set a dummy display for headless operation
    DOCKER_ARGS+=(
        "-e" "DISPLAY=:99"
    )
}

# Run the container
run_container() {
    echo "Starting container..."
    echo ""
    
    docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}:${IMAGE_TAG}" /bin/bash
}

# Main
build_docker_args
run_container


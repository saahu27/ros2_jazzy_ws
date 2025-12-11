#!/bin/bash

# Cross-platform entrypoint for ROS 2 + Gazebo container
# Handles display configuration for both Linux and macOS hosts

# Set ROS distribution
ROS_DISTRO="jazzy"

# ============================================
# Display Configuration
# ============================================

# Detect and configure display
configure_display() {
    # If DISPLAY is not set or empty, try to detect it
    if [ -z "$DISPLAY" ]; then
        # Check if running in Docker Desktop (macOS/Windows)
        if getent hosts host.docker.internal > /dev/null 2>&1; then
            export DISPLAY="host.docker.internal:0"
            echo "Display: Configured for Docker Desktop (host.docker.internal:0)"
        else
            # Fallback to :0 for native Linux
            export DISPLAY=":0"
            echo "Display: Configured for native Linux (:0)"
        fi
    else
        echo "Display: Using existing DISPLAY=$DISPLAY"
    fi
    
    # Ensure XDG_RUNTIME_DIR exists (needed by some GUI apps)
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"
}

# ============================================
# ROS 2 Environment Setup
# ============================================

setup_ros_environment() {
    # Set base paths
ROS_WS="/root/ros2_ws"
SHARED_ROS2="/root/shared/ros2"

    # Create shared directory if it doesn't exist
    mkdir -p "$SHARED_ROS2"
    
    # ROS Domain ID configuration
ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    echo "0" > "$ROS_DOMAIN_ID_FILE"
    echo "Created $ROS_DOMAIN_ID_FILE with default value 0"
fi

    # Set ROS_DOMAIN_ID if not already in .bashrc
if ! grep -q "export ROS_DOMAIN_ID" /root/.bashrc; then
  ros_domain_id=$(cat "$ROS_DOMAIN_ID_FILE")
  echo "export ROS_DOMAIN_ID=$ros_domain_id" >> /root/.bashrc
fi

export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

    # Source ROS 2 setup files
source /opt/ros/$ROS_DISTRO/setup.bash

    # Source workspace if it exists
    if [ -f "$ROS_WS/install/setup.bash" ]; then
        source "$ROS_WS/install/setup.bash"
    fi
}

# ============================================
# Workspace Build (if needed)
# ============================================

build_workspace_if_needed() {
    ROS_WS="/root/ros2_ws"
    
    # Only rebuild if install directory doesn't exist or is empty
    if [ ! -d "$ROS_WS/install" ] || [ -z "$(ls -A $ROS_WS/install 2>/dev/null)" ]; then
        echo "Building ROS 2 workspace..."
        cd "$ROS_WS"
colcon build
        source install/setup.bash
        cd
    fi
}

# ============================================
# Main Entrypoint
# ============================================

echo ""
echo "=== ROS 2 Jazzy + Gazebo Harmonic Container ==="
echo ""

# Configure display for GUI applications
configure_display

# Set up ROS 2 environment
setup_ros_environment

# Source bashrc for any additional user configuration
source /root/.bashrc

echo ""
echo "Environment ready!"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  DISPLAY: $DISPLAY"
echo ""

# Execute the command passed to the container
exec "$@"

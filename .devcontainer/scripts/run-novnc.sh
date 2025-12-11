#!/bin/bash

# noVNC-based Docker run script for macOS
# Access Gazebo via web browser at http://localhost:6080/vnc.html
# No VNC client needed - just a web browser!

set -e

SCRIPT_PATH=$(dirname $(realpath "$0"))
DEVCONTAINER_PATH=$(dirname "$SCRIPT_PATH")
WORKSPACE_ROOT=$(dirname "$DEVCONTAINER_PATH")

IMAGE_NAME="manipulation"
IMAGE_TAG="latest"

echo ""
echo "=== ROS 2 + Gazebo with noVNC (Web Access) ==="
echo ""
echo "After container starts:"
echo "  1. Open http://localhost:6080/vnc.html in your browser"
echo "  2. Click 'Connect'"
echo "  3. A desktop will appear - run 'gz sim' in the terminal"
echo ""

docker run -it --rm \
    --platform linux/amd64 \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    -p 6080:6080 \
    -v "${WORKSPACE_ROOT}/workspace:/root/ros2_ws/src" \
    -v "${WORKSPACE_ROOT}/results:/root/ros2_ws/src/puma560_ros2_moveit/results" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    bash -c '
        set -e
        echo "Installing noVNC and dependencies..."
        apt-get update -qq
        apt-get install -y -qq --no-install-recommends \
            xvfb x11vnc novnc python3-websockify \
            openbox xterm dbus-x11 fonts-dejavu 2>/dev/null
        
        echo "Starting virtual display..."
        export DISPLAY=:99
        Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset &
        sleep 2
        
        echo "Starting window manager..."
        openbox &
        sleep 1
        
        echo "Starting VNC server..."
        x11vnc -display :99 -forever -nopw -shared -rfbport 5900 -bg -o /tmp/x11vnc.log
        sleep 1
        
        # Find noVNC web directory
        NOVNC_DIR=$(find /usr -name "vnc.html" -type f 2>/dev/null | head -1 | xargs dirname 2>/dev/null || echo "/usr/share/novnc")
        echo "noVNC directory: $NOVNC_DIR"
        
        echo "Starting noVNC web server on port 6080..."
        websockify --web="$NOVNC_DIR" 6080 localhost:5900 &
        sleep 2
        
        echo ""
        echo "=============================================="
        echo "  noVNC Ready!"
        echo "  Open http://localhost:6080/vnc.html"
        echo "=============================================="
        echo ""
        
        # Source ROS
        source /opt/ros/jazzy/setup.bash
        source /root/ros2_ws/install/setup.bash 2>/dev/null || true
        
        # Set Mesa environment for software rendering
        export LIBGL_ALWAYS_SOFTWARE=1
        export MESA_GL_VERSION_OVERRIDE=3.3
        export MESA_GLSL_VERSION_OVERRIDE=330
        export GALLIUM_DRIVER=llvmpipe
        
        # Start a terminal in the virtual display
        xterm -geometry 120x35+10+10 -fa "DejaVu Sans Mono" -fs 11 -e "bash --rcfile <(echo \"source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash 2>/dev/null; echo Ready! Try: gz sim\")" &
        
        echo "Container ready. Press Ctrl+C to stop."
        echo ""
        
        # Keep container running
        tail -f /dev/null
    '

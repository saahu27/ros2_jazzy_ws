#!/bin/bash

# noVNC Startup Script for macOS Devcontainer
# This script starts all required services for GUI access via browser
# Access at: http://localhost:6080/vnc.html

DISPLAY_NUM=99
DISPLAY=:${DISPLAY_NUM}
export DISPLAY

LOG_DIR="/tmp/vnc-logs"
mkdir -p "$LOG_DIR"

log() {
    echo "[noVNC] $1"
}

# Function to check if a process is running
is_running() {
    pgrep -x "$1" > /dev/null 2>&1
}

# Function to wait for X server
wait_for_x() {
    local max_attempts=30
    local attempt=0
    while [ $attempt -lt $max_attempts ]; do
        if xdpyinfo -display $DISPLAY > /dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
        attempt=$((attempt + 1))
    done
    return 1
}

# Cleanup function
cleanup() {
    log "Cleaning up old processes..."
    pkill -9 Xvfb 2>/dev/null
    pkill -9 x11vnc 2>/dev/null
    pkill -9 websockify 2>/dev/null
    pkill -9 openbox 2>/dev/null
    sleep 1
    rm -f /tmp/.X${DISPLAY_NUM}-lock /tmp/.X11-unix/X${DISPLAY_NUM}
}

# Start Xvfb
start_xvfb() {
    if is_running Xvfb; then
        log "Xvfb already running"
        # Still wait for it to be ready
        sleep 2
        return 0
    fi
    
    log "Starting Xvfb on display $DISPLAY..."
    Xvfb $DISPLAY -screen 0 1920x1080x24 +extension GLX +render -noreset > "$LOG_DIR/xvfb.log" 2>&1 &
    
    if wait_for_x; then
        log "Xvfb started successfully"
        # Extra wait for X server to be fully ready
        sleep 3
        return 0
    else
        log "ERROR: Xvfb failed to start"
        return 1
    fi
}

# Start window manager
start_openbox() {
    if is_running openbox; then
        log "Openbox already running"
        return 0
    fi
    
    log "Starting Openbox window manager..."
    DISPLAY=$DISPLAY openbox > "$LOG_DIR/openbox.log" 2>&1 &
    sleep 1
    log "Openbox started"
}

# Start VNC server
start_x11vnc() {
    if is_running x11vnc; then
        log "x11vnc already running"
        return 0
    fi
    
    log "Starting x11vnc..."
    
    # Retry up to 5 times
    local attempt=0
    local max_attempts=5
    
    while [ $attempt -lt $max_attempts ]; do
        # Use -loop to auto-restart on crash, -repeat to allow key repeat
        x11vnc -display $DISPLAY -forever -loop -nopw -shared -rfbport 5900 -bg -o "$LOG_DIR/x11vnc.log" 2>&1
        sleep 2
        
        if is_running x11vnc; then
            log "x11vnc started on port 5900"
            return 0
        fi
        
        attempt=$((attempt + 1))
        log "x11vnc attempt $attempt failed, retrying..."
        sleep 2
    done
    
    log "ERROR: x11vnc failed to start after $max_attempts attempts"
    cat "$LOG_DIR/x11vnc.log" 2>/dev/null
    return 1
}

# Start noVNC websocket proxy
start_websockify() {
    if pgrep -f websockify > /dev/null 2>&1; then
        log "websockify already running"
        return 0
    fi
    
    log "Starting websockify on port 6080..."
    websockify --web=/usr/share/novnc 6080 localhost:5900 > "$LOG_DIR/websockify.log" 2>&1 &
    sleep 2
    
    if pgrep -f websockify > /dev/null 2>&1; then
        log "websockify started on port 6080"
        return 0
    else
        log "ERROR: websockify failed to start"
        return 1
    fi
}

# Start terminal
start_xterm() {
    log "Starting xterm..."
    DISPLAY=$DISPLAY xterm -geometry 120x35+50+50 -fa "DejaVu Sans Mono" -fs 11 > "$LOG_DIR/xterm.log" 2>&1 &
    sleep 1
    log "xterm started"
}

# Main execution
main() {
    # Wait a moment for container to be fully ready
    sleep 2
    
    log "=== Starting noVNC services ==="
    
    # Clean start
    cleanup
    
    # Start all services
    if ! start_xvfb; then
        log "Failed to start Xvfb, aborting"
        exit 1
    fi
    
    start_openbox
    
    if ! start_x11vnc; then
        log "Failed to start x11vnc, aborting"
        exit 1
    fi
    
    if ! start_websockify; then
        log "Failed to start websockify, aborting"
        exit 1
    fi
    
    start_xterm
    
    log ""
    log "=== noVNC Ready ==="
    log "Open http://localhost:6080/vnc.html in your browser"
    log ""
    
    # Add DISPLAY to bashrc for future terminals
    if ! grep -q "export DISPLAY=:${DISPLAY_NUM}" ~/.bashrc 2>/dev/null; then
        echo "export DISPLAY=:${DISPLAY_NUM}" >> ~/.bashrc
    fi
}

main "$@"


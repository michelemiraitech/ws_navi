#!/bin/bash

# DevContainer Post-Start Script
# Runs every time the container starts

set -e

echo "🚀 Starting development environment..."

# Ensure VNC server is running
if ! pgrep -f "Xvnc :1" > /dev/null; then
    echo "🖥️  Starting VNC server..."
    sudo -u ros vncserver :1 -geometry 1920x1080 -depth 24 > /dev/null 2>&1 || true
fi

# Ensure noVNC is running
if ! pgrep -f "novnc_proxy" > /dev/null; then
    echo "🌐 Starting noVNC web server..."
    /opt/noVNC/utils/novnc_proxy --vnc localhost:5901 --listen 6901 > /dev/null 2>&1 &
fi

# Source workspace if it exists
if [ -f "/workspace/install/setup.bash" ]; then
    echo "📦 Workspace found - sourcing environment..."
    cd /workspace
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
fi

echo "✅ Development environment ready!"

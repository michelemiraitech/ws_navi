#!/bin/bash

# DevContainer Entrypoint Script
# This script initializes the development environment

set -e

echo "🚀 Starting ROS 2 Vehicle Navigation DevContainer..."

# Start VNC server for GUI applications
echo "🖥️  Starting VNC server..."
sudo -u ros vncserver :1 -geometry 1920x1080 -depth 24 > /dev/null 2>&1 || true

# Start noVNC web server
echo "🌐 Starting noVNC web server..."
/opt/noVNC/utils/novnc_proxy --vnc localhost:5901 --listen 6901 > /dev/null 2>&1 &

# Source ROS 2 environment
echo "🤖 Setting up ROS 2 environment..."
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source Python virtual environment
echo "🐍 Activating Python virtual environment..."
source /opt/venv/bin/activate

# Change to workspace directory
cd ${WORKSPACE}

# Source workspace if it exists
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    echo "📦 Sourcing workspace..."
    source ${WORKSPACE}/install/setup.bash
fi

# Update rosdep if workspace has packages
if [ -d "${WORKSPACE}/src" ] && [ -n "$(ls -A ${WORKSPACE}/src)" ]; then
    echo "📋 Updating rosdep dependencies..."
    rosdep update > /dev/null 2>&1 || true
    rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1 || true
fi

# Switch to ros user
echo "👤 Switching to ros user..."
exec gosu ros "$@"

#!/bin/bash

# DevContainer Initialize Script
# Runs on the host before the container is created

set -e

echo "🔧 Initializing DevContainer environment..."

# Ensure .devcontainer directory has proper permissions
chmod +x .devcontainer/*.sh

# Create .env file if it doesn't exist
if [ ! -f ".devcontainer/.env" ]; then
    echo "📝 Creating .env file..."
    cat > .devcontainer/.env << EOF
# DevContainer Environment Variables
ROS_DISTRO=jazzy
WORKSPACE=/workspace
DISPLAY=:1
QT_X11_NO_MITSHM=1
LIBGL_ALWAYS_SOFTWARE=1
EOF
fi

echo "✅ DevContainer initialization completed!"

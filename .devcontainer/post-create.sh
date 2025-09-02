#!/bin/bash

# DevContainer Post-Create Script
# Runs after the container is created and VS Code connects

set -e

echo "🔧 Running post-create setup..."

# Ensure we're in the workspace
cd /workspace

# Setup Python environment if pyproject.toml exists
if [ -f "pyproject.toml" ]; then
    echo "🐍 Setting up Python environment with uv..."
    source /opt/venv/bin/activate
    
    # Install project dependencies
    if [ -f "pyproject.toml" ]; then
        uv pip install -e . || echo "⚠️  Could not install project in editable mode"
    fi
fi

# Source ROS 2 environment
echo "🤖 Sourcing ROS 2 environment..."
source /opt/ros/jazzy/setup.bash

# Install/update ROS dependencies
if [ -d "src" ] && [ -n "$(ls -A src)" ]; then
    echo "📦 Installing ROS dependencies..."
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y || echo "⚠️  Some dependencies may not be available"
fi

# Setup git safe directory
echo "🔒 Setting up git safe directory..."
git config --global --add safe.directory /workspace

# Setup pre-commit hooks if they exist
if [ -f ".pre-commit-config.yaml" ]; then
    echo "🪝 Installing pre-commit hooks..."
    source /opt/venv/bin/activate
    pre-commit install || echo "⚠️  Could not install pre-commit hooks"
fi

# Make scripts executable
echo "🛠️  Making scripts executable..."
chmod +x ci/*.sh 2>/dev/null || true
chmod +x setup_python_env.sh 2>/dev/null || true

# Build workspace if packages exist
if [ -d "src" ] && [ -n "$(ls -A src)" ]; then
    echo "🏗️  Building workspace..."
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install 2>/dev/null || echo "⚠️  Initial build failed - this is normal for a new workspace"
fi

# Setup Dagger
echo "🔧 Setting up Dagger..."
dagger version || echo "⚠️  Dagger not available"

echo "✅ Post-create setup completed!"
echo ""
echo "🎉 Welcome to the ROS 2 Vehicle Navigation DevContainer!"
echo ""
echo "📋 Available commands:"
echo "  • colcon build              - Build the ROS 2 workspace"
echo "  • ./ci/run.sh build-and-test - Run CI pipeline locally"
echo "  • ./ci/dev.sh setup        - Setup development environment"
echo "  • uv run ruff check .      - Run linting"
echo "  • ros2 launch ...          - Launch ROS 2 nodes"
echo ""
echo "🌐 GUI Access:"
echo "  • VNC: localhost:5901 (password: vncpassword)"
echo "  • noVNC Web: http://localhost:6901"
echo ""
echo "📚 Documentation:"
echo "  • README.md - Project documentation"
echo "  • GITHUB_ACTIONS_SETUP.md - CI/CD setup guide"

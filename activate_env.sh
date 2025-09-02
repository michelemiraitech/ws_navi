#!/bin/bash

# ROS 2 Vehicle Navigation Environment Setup
# This script activates both ROS 2 and Python virtual environment

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== Activating ROS 2 Vehicle Navigation Environment ===${NC}"

# First, source ROS 2 Jazzy (this sets up the system Python path for ROS packages)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS 2 Jazzy sourced${NC}"
else
    echo -e "${YELLOW}⚠ ROS 2 Jazzy not found at /opt/ros/jazzy${NC}"
fi

# Then activate Python virtual environment (with --system-site-packages)
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
    echo -e "${GREEN}✓ Python virtual environment activated${NC}"
else
    echo -e "${YELLOW}⚠ Python virtual environment not found. Run ./setup_python_env.sh${NC}"
fi

# Verify ROS 2 Python packages are accessible
if python -c "import rclpy" 2>/dev/null; then
    echo -e "${GREEN}✓ ROS 2 Python packages accessible${NC}"
else
    echo -e "${YELLOW}⚠ ROS 2 Python packages not accessible${NC}"
fi

# Source workspace if built
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
    echo -e "${GREEN}✓ ROS 2 workspace sourced${NC}"
else
    echo -e "${YELLOW}⚠ ROS 2 workspace not built yet. Run 'colcon build' first.${NC}"
fi

# Set environment variables
export PYTHONPATH="$PYTHONPATH:$(pwd)/src"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/vehicle_simulation/models
export ROS_DOMAIN_ID=42

echo -e "${GREEN}🎉 Environment activated successfully!${NC}"
echo -e "${YELLOW}Configuration:${NC}"
echo "  Python: $(which python)"
echo "  ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "  PYTHONPATH includes ROS packages: $(python -c 'import sys; print("rclpy" in str(sys.path))' 2>/dev/null || echo 'false')"
echo ""
echo -e "${YELLOW}Available commands:${NC}"
echo "  colcon build                  # Build ROS workspace"
echo "  ros2 launch ...              # Launch ROS nodes"
echo "  rviz2                        # Launch RViz"
echo "  uv pip list                  # List Python packages"
echo "  uv pip install <package>     # Install new package"
echo "  pytest                       # Run Python tests"
echo "  ruff check .                 # Lint code"
echo "  ruff format .                # Format code"

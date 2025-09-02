#!/bin/bash

# ROS 2 Vehicle Workspace Setup Script

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up ROS 2 Vehicle Workspace...${NC}"

# Source ROS 2 Jazzy
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f "./install/setup.bash" ]; then
    echo -e "${YELLOW}Sourcing workspace...${NC}"
    source ./install/setup.bash
else
    echo -e "${YELLOW}Workspace not built yet. Run 'colcon build' first.${NC}"
fi

# Set environment variables
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/vehicle_simulation/models
export ROS_DOMAIN_ID=42

echo -e "${GREEN}Environment setup complete!${NC}"
echo -e "${YELLOW}Available launch commands:${NC}"
echo "  # Build workspace:"
echo "  colcon build"
echo ""
echo "  # Launch simulation with SLAM:"
echo "  ros2 launch vehicle_bringup vehicle_bringup.launch.py slam:=true"
echo ""
echo "  # Launch simulation with navigation (requires map):"
echo "  ros2 launch vehicle_bringup vehicle_bringup.launch.py slam:=false"
echo ""
echo "  # Launch only Gazebo simulation:"
echo "  ros2 launch vehicle_simulation gazebo.launch.py"
echo ""
echo "  # Launch only navigation stack:"
echo "  ros2 launch vehicle_navigation navigation.launch.py"

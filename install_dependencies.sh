#!/bin/bash

# Installation script for ROS 2 Vehicle Navigation Workspace dependencies

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS 2 Vehicle Navigation Workspace Installation ===${NC}"

# Check if running on Ubuntu
if [[ ! -f /etc/lsb-release ]]; then
    echo -e "${RED}This script is designed for Ubuntu. Please install dependencies manually.${NC}"
    exit 1
fi

# Source ROS 2
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# Update package list
echo -e "${YELLOW}Updating package list...${NC}"
sudo apt update

# Install base ROS 2 dependencies
echo -e "${YELLOW}Installing base ROS 2 packages...${NC}"
sudo apt install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-urdf

# Install Gazebo and simulation packages
echo -e "${YELLOW}Installing Gazebo simulation packages...${NC}"
sudo apt install -y \
    ros-jazzy-gazebo-ros \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-plugins

# Install Nav2 packages
echo -e "${YELLOW}Installing Nav2 navigation packages...${NC}"
sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-smoother \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-velocity-smoother

# Install localization packages
echo -e "${YELLOW}Installing localization packages...${NC}"
sudo apt install -y \
    ros-jazzy-robot-localization

# Install SLAM packages
echo -e "${YELLOW}Installing SLAM packages...${NC}"
sudo apt install -y \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-cartographer-rviz \
    ros-jazzy-slam-toolbox

# Install control packages
echo -e "${YELLOW}Installing control packages...${NC}"
sudo apt install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster

# Install visualization packages
echo -e "${YELLOW}Installing visualization packages...${NC}"
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-nav2-rviz-plugins

# Install additional utilities
echo -e "${YELLOW}Installing additional utilities...${NC}"
sudo apt install -y \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-sensor-msgs

echo -e "${GREEN}=== Installation Complete! ===${NC}"
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Build the workspace:"
echo "   cd /opt/ws_navi"
echo "   colcon build"
echo ""
echo "2. Source the workspace:"
echo "   source install/setup.bash"
echo ""
echo "3. Launch the vehicle simulation:"
echo "   ros2 launch vehicle_bringup vehicle_bringup.launch.py slam:=true"
echo ""
echo -e "${GREEN}Happy navigation! 🚗${NC}"

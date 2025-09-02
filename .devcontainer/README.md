# 🐳 DevContainer Setup for ROS 2 Vehicle Navigation

This directory contains the development container configuration for the ROS 2 Vehicle Navigation project, providing a complete, consistent development environment.

## 🚀 Features

### 🤖 **Complete ROS 2 Environment**
- **Official ROS 2 Jazzy** base image from Open Robotics Foundation
- **All navigation packages** pre-installed (Nav2, Cartographer, robot_localization)
- **Gazebo simulation** ready to use
- **Development tools** (colcon, rosdep, vcstool)

### 🐍 **Modern Python Development**
- **Python virtual environment** with uv package manager
- **Fast linting and formatting** with ruff
- **Testing framework** with pytest
- **Pre-commit hooks** for code quality

### 🖥️ **GUI Application Support**
- **VNC server** for GUI applications (RViz, Gazebo, etc.)
- **noVNC web interface** accessible at http://localhost:6901
- **X11 forwarding** support
- **Hardware acceleration** for graphics

### 🔧 **Development Tools**
- **VS Code extensions** for ROS, Python, C++, and more
- **Dagger CI/CD** tools for consistent testing
- **Git integration** with proper safe directory setup
- **SSH key forwarding** from host

## 🚀 Quick Start

### Prerequisites
- **VS Code** with Remote-Containers extension
- **Docker** installed and running
- **Git** for cloning the repository

### 1. Open in DevContainer
```bash
# Clone the repository
git clone <your-repo-url>
cd vehicle-navigation

# Open in VS Code
code .

# VS Code will prompt to "Reopen in Container" - click it!
# Or use Command Palette: "Dev Containers: Reopen in Container"
```

### 2. Wait for Setup
The container will automatically:
- 🐳 Build the Docker image with ROS 2 Jazzy
- 📦 Install all dependencies
- 🐍 Setup Python virtual environment
- 🏗️ Build the ROS workspace
- 🔧 Configure development tools

### 3. Start Developing!
```bash
# Terminal opens in /workspace with everything ready

# Build the workspace
colcon build

# Run simulation
ros2 launch vehicle_simulation gazebo.launch.py

# Run CI pipeline (same as GitHub Actions)
./ci/run.sh build-and-test
```

## 🖥️ GUI Applications

### VNC Access
- **Direct VNC**: `localhost:5901` (password: `vncpassword`)
- **Web VNC**: http://localhost:6901 (no password needed)

### Running GUI Applications
```bash
# RViz for visualization
rviz2

# Gazebo simulation
gazebo

# RQT tools
rqt
```

## 🔧 Container Configuration

### Key Files
- **`devcontainer.json`** - Main configuration with VS Code settings
- **`Dockerfile`** - Container image definition with ROS 2 Jazzy
- **`post-create.sh`** - Setup script after container creation
- **`post-start.sh`** - Startup script for each container start
- **`entrypoint.sh`** - Container entry point with services
- **`.env`** - Environment variables

### Container Features
- **Base Image**: `osrf/ros:jazzy-desktop-full`
- **User**: `ros` (non-root with sudo access)
- **Python**: Virtual environment at `/opt/venv`
- **Workspace**: `/workspace` (mounted from host)
- **Services**: VNC server, noVNC web server

### Port Forwarding
- **5901** - VNC server
- **6901** - noVNC web interface
- **8080** - Web interfaces
- **9090** - ROS bridge

## 🛠️ Development Workflow

### 1. Code Development
```bash
# Edit code in VS Code (with full IntelliSense)
# Files are automatically synced with host

# Format and lint code
uv run ruff format .
uv run ruff check . --fix

# Run tests
pytest
```

### 2. ROS Development
```bash
# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Run nodes
ros2 launch vehicle_bringup navigation.launch.py

# Debug with tools
ros2 topic list
ros2 node list
rviz2
```

### 3. CI/CD Testing
```bash
# Run same pipeline as GitHub Actions
./ci/run.sh build-and-test

# Quick linting
./ci/run.sh lint

# Test different configurations
ROS_DISTRO=humble ./ci/run.sh build-and-test
```

## 🔍 Troubleshooting

### Container Issues
```bash
# Rebuild container
# Command Palette: "Dev Containers: Rebuild Container"

# View container logs
docker logs <container-name>

# Check container status
docker ps
```

### GUI Not Working
```bash
# Check VNC server
ps aux | grep vnc

# Restart VNC server
vncserver -kill :1
vncserver :1 -geometry 1920x1080
```

### Permission Issues
```bash
# Fix workspace permissions
sudo chown -R ros:ros /workspace

# Reset git safe directory
git config --global --add safe.directory /workspace
```

### Build Issues
```bash
# Clean build
rm -rf build install log
colcon build

# Update dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 📦 Installed Packages

### ROS 2 Packages
- `nav2-bringup` - Navigation stack
- `cartographer-ros` - SLAM
- `robot-localization` - Sensor fusion
- `gazebo-ros-pkgs` - Simulation
- `rviz2` - Visualization
- `tf2-tools` - Transform debugging

### Python Packages
- `ruff` - Fast linting and formatting
- `pytest` - Testing framework
- `dagger-io` - CI/CD toolkit
- `numpy` - Scientific computing
- `matplotlib` - Plotting

### Development Tools
- `colcon` - Build tool
- `rosdep` - Dependency management
- `vcstool` - Version control
- `pre-commit` - Git hooks

## 🎯 Benefits

### ✅ **Consistency**
- Same environment for all developers
- Matches CI/CD pipeline exactly
- No "works on my machine" issues

### ✅ **Speed** 
- Pre-built image with all dependencies
- Fast container startup
- Incremental builds with volume mounts

### ✅ **Isolation**
- Clean environment for each project
- No conflicts with host system
- Easy cleanup and reset

### ✅ **Complete Setup**
- ROS 2, Python, GUI, tools all ready
- VS Code extensions pre-installed
- CI/CD tools included

---

**🎉 Happy Development in your ROS 2 DevContainer!** 🚗💨

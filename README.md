# Vehicle Navigation Project

A comprehensive ROS 2 navigation system for autonomous vehicles with GPS/IMU localization, Nav2 navigation stack, Gazebo simulation, and Cartographer SLAM integration.

## 🚗 Project Overview

This project implements a complete autonomous vehicle navigation system using:

- **ROS 2 Jazzy** - Latest ROS distribution
- **Nav2** - Navigation stack with DWB controller and AMCL localization
- **Gazebo** - Physics-based simulation environment
- **GPS/IMU Integration** - Multi-sensor fusion with robot_localization
- **Cartographer SLAM** - Google's SLAM solution for mapping
- **Modern Python Tooling** - uv for dependency management, ruff for linting
- **Dagger CI/CD** - Container-based pipeline with multi-ROS distro support
- **DevContainer Support** - Complete VS Code development environment with GUI support

## 📦 Package Structure

```
src/
├── vehicle_bringup/          # Main launch files and integration
├── vehicle_control/          # Vehicle control interfaces
├── vehicle_description/      # URDF models and robot description
├── vehicle_localization/     # GPS/IMU/odometry sensor fusion
├── vehicle_navigation/       # Nav2 configuration and parameters
└── vehicle_simulation/       # Gazebo simulation files
```

## Features

### Navigation
- **Nav2**: Complete navigation stack with path planning, obstacle avoidance, and recovery behaviors
- **Costmap filters**: Custom filter plugins for enhanced obstacle detection
- **Behavior trees**: Configurable navigation behaviors

### Localization
- **Multi-sensor fusion**: GPS, IMU, and wheel odometry fusion using Extended Kalman Filter
- **Robot localization**: EKF-based state estimation
- **Transform management**: Proper TF tree management

### SLAM
- **Cartographer**: Google's Cartographer SLAM integration
- **Map building**: Real-time mapping capabilities
- **Loop closure**: Automatic loop closure detection

### Simulation
- **Gazebo integration**: Full physics simulation
- **Sensor simulation**: Lidar, GPS, IMU, and odometry sensors
- **Realistic environment**: Test world with obstacles

## 🚀 Quick Start

### Option 1: DevContainer (Recommended)
Use VS Code with the provided devcontainer for the fastest setup:

```bash
# Clone the repository
git clone <your-repo-url>
cd vehicle-navigation

# Open in VS Code
code .

# Click "Reopen in Container" when prompted
# Everything will be automatically configured!
```

The devcontainer provides:
- 🤖 **ROS 2 Jazzy** pre-installed with all dependencies
- 🐍 **Python environment** with uv and ruff
- 🖥️ **GUI support** via VNC/noVNC for RViz and Gazebo
- 🔧 **Development tools** and VS Code extensions
- 🚀 **CI/CD tools** (same as GitHub Actions)

### Option 2: Local Installation

### Option 2: Local Installation

#### Prerequisites
- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Docker (for Dagger CI pipeline)
- Python 3.10+

#### Setup Steps
#### Setup Steps
```bash
# Navigate to workspace
cd /opt/ws_navi

# Setup Python environment
./setup_python_env.sh

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Run Simulation (Both Options)
### Run Simulation (Both Options)
```bash
# Launch Gazebo simulation with vehicle
ros2 launch vehicle_simulation gazebo.launch.py

# In another terminal, launch navigation
ros2 launch vehicle_bringup navigation.launch.py

# In another terminal, launch SLAM (optional)
ros2 launch vehicle_bringup slam.launch.py
```

### Control the Vehicle (Both Options)
### Control the Vehicle (Both Options)
```bash
# Launch RViz for visualization and goal setting
rviz2 -d src/vehicle_navigation/rviz/navigation.rviz

# Or use command line to send goals
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

**DevContainer Users**: GUI applications automatically work via web browser at http://localhost:6901

### Test Your Changes (Both Options)
```bash
# Run the same CI pipeline that GitHub Actions uses
./ci/run.sh build-and-test

# Quick linting check
./ci/run.sh lint
```

## Python Development Environment

This project uses **uv** for fast and reliable Python dependency management alongside ROS 2.

### Python Environment Setup
```bash
# Setup Python virtual environment with uv
./setup_python_env.sh

# Activate environment (includes ROS 2 + Python venv)
./activate_env.sh
```

### Python Dependencies
The project includes several Python utilities:
- **GPS conversion**: UTM coordinate conversion for localization
- **Navigation analysis**: Performance monitoring and path analysis
- **Setup tools**: System checks and workspace management
- **Testing framework**: Comprehensive test suite with pytest

## 🧪 Development & Testing

### Linting and Code Quality
```bash
# Run linting with ruff (fast Rust-based linter)
uv run ruff check .

# Auto-fix issues
uv run ruff check . --fix

# Format code
uv run ruff format .
```

### CI/CD Pipeline with GitHub Actions
This project uses **Dagger** to provide **identical CI pipelines** locally and in GitHub Actions:

```bash
# Run complete CI pipeline locally (same as GitHub Actions)
./ci/run.sh build-and-test

# Run only linting
./ci/run.sh lint

# Run with different ROS distro
ROS_DISTRO=humble ./ci/run.sh build-and-test

# Development utilities
./ci/dev.sh setup    # Setup development environment
./ci/dev.sh test     # Run full pipeline locally
./ci/dev.sh lint     # Run only linting
./ci/dev.sh docs     # Build documentation
./ci/dev.sh clean    # Clean artifacts
```

#### GitHub Actions Features
- **🔄 Identical Environment**: Local Dagger pipeline = CI pipeline
- **⚡ Matrix Testing**: Multiple ROS distros (Jazzy, Humble) + Python versions
- **📋 Parallel Jobs**: Fast feedback with linting, build, test, and security scans
- **📚 Auto-Documentation**: Deploys to GitHub Pages on main branch pushes
- **🔒 Security Scanning**: Trivy vulnerability detection
- **📦 Artifact Upload**: Test results, coverage reports, and build logs
- **🎯 Manual Triggers**: Workflow dispatch with custom parameters

#### Local Development Matches CI
When you run `./ci/run.sh`, you get the **exact same environment** as GitHub Actions:
- Same ROS Docker containers
- Same dependency versions
- Same test procedures
- Same linting rules
- Same build process

This eliminates "works on my machine" issues!

## Detailed Usage

### Building the workspace
```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### Running simulation only
```bash
ros2 launch vehicle_simulation gazebo.launch.py world:=test_world.world
```

### Running navigation stack only
```bash
ros2 launch vehicle_navigation navigation.launch.py use_sim_time:=true
```

### Running SLAM with Cartographer
```bash
ros2 launch vehicle_bringup vehicle_bringup.launch.py slam:=true
```

## Configuration

### Navigation Parameters
Navigation parameters are located in `vehicle_navigation/config/`:
- `nav2_params.yaml`: Main Nav2 configuration
- `costmap_2d.yaml`: Costmap configuration with filter plugins
- `controller.yaml`: DWB controller parameters
- `planner_server.yaml`: Path planner configuration

### Localization Parameters
Localization parameters are in `vehicle_localization/config/`:
- `ekf.yaml`: Extended Kalman Filter configuration for sensor fusion

### SLAM Parameters
Cartographer configuration is in `vehicle_bringup/config/`:
- `cartographer.lua`: Cartographer SLAM parameters

## Vehicle Configuration

### Sensors
The vehicle is equipped with:
- **Lidar**: 2D laser scanner for obstacle detection
- **IMU**: Inertial measurement unit for orientation and angular velocity
- **GPS**: Global positioning system for absolute localization
- **Wheel encoders**: Differential drive odometry

### Physical Properties
- **Dimensions**: 1.2m x 0.6m x 0.3m
- **Wheel configuration**: 4-wheel differential drive
- **Sensor mounting**: Lidar on top, IMU and GPS on the base

## Dependencies

### Required ROS 2 packages
- nav2_bringup
- nav2_*
- cartographer_ros
- robot_localization
- gazebo_ros_pkgs
- urdf
- xacro

### Installation
```bash
sudo apt update
sudo apt install ros-jazzy-nav2-bringup \
                 ros-jazzy-nav2-common \
                 ros-jazzy-cartographer \
                 ros-jazzy-cartographer-ros \
                 ros-jazzy-robot-localization \
                 ros-jazzy-gazebo-ros-pkgs \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-robot-state-publisher \
                 ros-jazzy-xacro
```

## Troubleshooting

### Common Issues

1. **Transforms not available**
   - Check that robot_state_publisher is running
   - Verify URDF is correct
   - Check TF tree with `ros2 run tf2_tools view_frames`

2. **Navigation not working**
   - Ensure map is loaded correctly
   - Check costmap configuration
   - Verify sensor data is being received

3. **SLAM not mapping**
   - Check Lidar data is being published
   - Verify Cartographer configuration
   - Ensure proper odometry data

### Debugging Commands
```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor navigation status
ros2 topic echo /navigation_result

# Check sensor data
ros2 topic echo /scan
ros2 topic echo /imu
ros2 topic echo /fix
```

## Development

### Adding Custom Costmap Filters
1. Create filter plugin in `vehicle_navigation/src/`
2. Update `costmap_2d.yaml` configuration
3. Add plugin to CMakeLists.txt and package.xml

### Extending Localization
1. Add sensor configurations to `ekf.yaml`
2. Create sensor drivers if needed
3. Update launch files to include new sensors

### Custom Behavior Trees
1. Create custom BT nodes in `vehicle_navigation/`
2. Configure behavior trees in `behavior_trees/`
3. Update bt_navigator configuration

## License

Apache 2.0 License

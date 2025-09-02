# 🚗 Vehicle Navigation Project

[![CI Status](https://github.com/username/vehicle-navigation/workflows/ROS%202%20Vehicle%20Navigation%20CI/badge.svg)](https://github.com/username/vehicle-navigation/actions)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://username.github.io/vehicle-navigation)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A comprehensive ROS 2 navigation system for autonomous vehicles with GPS/IMU localization, Nav2 navigation stack, Gazebo simulation, and Cartographer SLAM integration.

## 🚀 Key Features

- **🤖 Complete ROS 2 Integration** - Full workspace with 6 specialized packages
- **🗺️ Advanced Navigation** - Nav2 stack with DWB controller and behavior trees
- **📍 Multi-Sensor Fusion** - GPS, IMU, and odometry integration with EKF
- **🎮 Realistic Simulation** - Gazebo physics with sensor plugins
- **🎯 SLAM Capabilities** - Cartographer for real-time mapping
- **⚡ Modern CI/CD** - Dagger-based pipeline with GitHub Actions
- **🐍 Modern Python Tools** - uv for dependencies, ruff for fast linting

## 🚀 Quick Start

### Prerequisites
- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Docker (for Dagger CI pipeline)
- Python 3.10+

### 1-Minute Setup
```bash
# Clone the repository
git clone https://github.com/username/vehicle-navigation.git
cd vehicle-navigation

# Setup environment (installs uv, creates Python env)
./setup_python_env.sh

# Build the workspace
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Run Simulation
```bash
# Terminal 1: Start Gazebo simulation
ros2 launch vehicle_simulation gazebo.launch.py

# Terminal 2: Launch navigation stack
ros2 launch vehicle_bringup navigation.launch.py

# Terminal 3: Open RViz for visualization
rviz2 -d src/vehicle_navigation/rviz/navigation.rviz
```

## 🧪 Development & Testing

### Local CI Pipeline
The project uses **Dagger** for consistent CI/CD that runs **identical pipelines** locally and in GitHub Actions:

```bash
# Run complete CI pipeline (same as GitHub Actions)
./ci/run.sh build-and-test

# Quick linting check
./ci/run.sh lint

# Test with different ROS distro
ROS_DISTRO=humble ./ci/run.sh build-and-test

# Build documentation
./ci/run.sh docs
```

### Development Tools
```bash
# Setup development environment
./ci/dev.sh setup

# Run full pipeline locally
./ci/dev.sh test

# Format and lint code (super fast with ruff!)
uv run ruff format .
uv run ruff check . --fix
```

## 🏗️ CI/CD Pipeline

### GitHub Actions Integration
The project includes a comprehensive CI pipeline that:

- **🔍 Linting** - Fast code quality checks with ruff
- **🏗️ Multi-Matrix Builds** - Tests ROS Jazzy & Humble with Python 3.10/3.11
- **🧪 Integration Tests** - Full navigation stack testing
- **📚 Documentation** - Auto-deploys to GitHub Pages
- **🔒 Security Scanning** - Vulnerability detection with Trivy

### Pipeline Features
- ✅ **Same Environment**: Local and CI use identical Dagger containers
- ✅ **Fast Feedback**: Parallel linting and build jobs
- ✅ **Multi-ROS Support**: Parameterized testing across distributions
- ✅ **Artifact Upload**: Test results and coverage reports
- ✅ **Security First**: Automated vulnerability scanning

## 📦 Package Architecture

```
src/
├── vehicle_bringup/          # 🚀 Main launch files and system integration
├── vehicle_control/          # 🎮 Vehicle control interfaces
├── vehicle_description/      # 🤖 URDF models with GPS/IMU/Lidar sensors
├── vehicle_localization/     # 📍 Multi-sensor fusion (GPS+IMU+Odometry)
├── vehicle_navigation/       # 🗺️ Nav2 stack configuration & costmap filters
└── vehicle_simulation/       # 🎮 Gazebo worlds and sensor plugins
```

### Key Technologies
- **Nav2** - Path planning, obstacle avoidance, recovery behaviors
- **Cartographer** - Google's SLAM for real-time mapping
- **robot_localization** - EKF-based sensor fusion
- **Gazebo** - Physics simulation with realistic sensor modeling
- **uv** - Ultra-fast Python package management
- **ruff** - Lightning-fast linting (10-100x faster than black+flake8)

## 🔧 Configuration

### Vehicle Sensors
- **📡 GPS** - Global positioning with UTM conversion
- **🧭 IMU** - 9-DOF inertial measurement (orientation, acceleration)
- **👁️ Lidar** - 360° laser scanner for mapping and obstacles
- **⚙️ Wheel Encoders** - Differential drive odometry

### Navigation Stack
- **DWB Controller** - Dynamic window approach for local planning
- **NavFn Planner** - Global path planning with Dijkstra
- **AMCL Localizer** - Adaptive Monte Carlo localization
- **Costmap Filters** - Advanced obstacle and constraint handling

## 📚 Documentation

- **📖 [Full Documentation](https://username.github.io/vehicle-navigation)** - Auto-generated from code
- **🚀 [Quick Start Guide](docs/quick-start.md)** - Get running in 5 minutes
- **🔧 [Configuration Guide](docs/configuration.md)** - Customize navigation parameters
- **🧪 [Testing Guide](docs/testing.md)** - Run tests and CI pipeline
- **🤝 [Contributing Guide](CONTRIBUTING.md)** - Development workflow

## 🤝 Contributing

We welcome contributions! The project uses modern development practices:

1. **🍴 Fork** the repository
2. **🌿 Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **✅ Test** locally with `./ci/run.sh build-and-test`
4. **📝 Commit** changes (`git commit -m 'Add amazing feature'`)
5. **🚀 Push** to branch (`git push origin feature/amazing-feature`)
6. **🔄 Open** a Pull Request

### Code Quality
- ✅ **Linting**: Use `ruff` for fast code formatting
- ✅ **Testing**: CI pipeline runs automatically on PRs
- ✅ **Documentation**: Keep README and docs updated
- ✅ **ROS Standards**: Follow ROS 2 best practices

## 📄 License

This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.

## 🎯 Roadmap

- [ ] 🧠 Advanced path planning algorithms (A*, RRT*)
- [ ] 🤖 Multi-robot coordination and swarm navigation
- [ ] 🔮 Deep learning integration for perception
- [ ] ☁️ Cloud-based monitoring and fleet management
- [ ] 🔬 Hardware-in-the-loop testing framework
- [ ] 📊 Performance benchmarking and metrics

## ⭐ Star History

[![Star History Chart](https://api.star-history.com/svg?repos=username/vehicle-navigation&type=Date)](https://star-history.com/#username/vehicle-navigation&Date)

---

<p align="center">
  <strong>🚗💨 Happy Navigating!</strong><br>
  Built with ❤️ using ROS 2, Nav2, and modern DevOps practices
</p>

#!/bin/bash

# Setup script for Python virtual environment using uv
# This script sets up the Python development environment for the ROS 2 vehicle project

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Setting up Python Virtual Environment with uv ===${NC}"

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    echo -e "${YELLOW}Installing uv...${NC}"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    source $HOME/.cargo/env
fi

# Ensure we're in the workspace directory
cd /opt/ws_navi

# Remove existing virtual environment if it exists
if [ -d ".venv" ]; then
    echo -e "${YELLOW}Removing existing virtual environment...${NC}"
    rm -rf .venv
fi

# Create virtual environment with uv that can access system packages
echo -e "${YELLOW}Creating virtual environment with Python 3.12 (matching ROS 2 Jazzy)...${NC}"
uv venv --python 3.12 --system-site-packages

# Activate virtual environment
echo -e "${YELLOW}Activating virtual environment...${NC}"
source .venv/bin/activate

# Verify ROS 2 Python packages are accessible
echo -e "${YELLOW}Checking ROS 2 Python package access...${NC}"
if python -c "import rclpy" 2>/dev/null; then
    echo -e "${GREEN}✓ ROS 2 Python packages accessible${NC}"
else
    echo -e "${RED}✗ ROS 2 Python packages not accessible${NC}"
    echo -e "${YELLOW}Make sure ROS 2 Jazzy is installed and sourced${NC}"
fi

# Install dependencies (excluding ROS packages which are system-installed)
echo -e "${YELLOW}Installing Python dependencies...${NC}"
# Install core scientific packages
uv pip install \
    "numpy>=1.24.0" \
    "scipy>=1.10.0" \
    "pyproj>=3.4.0" \
    "geopy>=2.3.0" \
    "utm>=0.7.0" \
    "pandas>=2.0.0" \
    "matplotlib>=3.6.0" \
    "seaborn>=0.12.0" \
    "pyyaml>=6.0" \
    "click>=8.0.0" \
    "rich>=13.0.0" \
    "transforms3d>=0.4.1" \
    "pytest>=7.0.0" \
    "pytest-cov>=4.0.0" \
    "ruff>=0.0.290" \
    "mypy>=1.0.0"

# Install development dependencies
echo -e "${YELLOW}Installing development dependencies...${NC}"
uv pip install \
    "pre-commit>=3.0.0" \
    "sphinx>=6.0.0" \
    "sphinx-rtd-theme>=1.2.0" \
    "jupyter>=1.0.0" \
    "ipykernel>=6.0.0"

# Install simulation dependencies
echo -e "${YELLOW}Installing simulation dependencies...${NC}"
uv pip install "opencv-python>=4.7.0"

# Install project in editable mode (without dependencies since we installed them manually)
echo -e "${YELLOW}Installing project in editable mode...${NC}"
uv pip install -e . --no-deps

# Create activation script
echo -e "${YELLOW}Creating activation script...${NC}"
cat > activate_env.sh << 'EOF'
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
EOF

chmod +x activate_env.sh

# Create development tools
echo -e "${YELLOW}Setting up development tools...${NC}"

# Create pre-commit configuration
cat > .pre-commit-config.yaml << 'EOF'
repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.4.0
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-yaml
      - id: check-added-large-files
      - id: check-merge-conflict

  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.0.290
    hooks:
      # Run the linter.
      - id: ruff
        args: [ --fix, --exit-non-zero-on-fix ]
      # Run the formatter.
      - id: ruff-format

  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v1.5.1
    hooks:
      - id: mypy
        additional_dependencies: [types-all]
        files: ^src/
EOF

# Install pre-commit hooks
echo -e "${YELLOW}Installing pre-commit hooks...${NC}"
uv pip install pre-commit
pre-commit install

# Create Python source directory structure
echo -e "${YELLOW}Creating Python package structure...${NC}"
mkdir -p src/vehicle_tools/{gps,navigation,control,simulation,utils}

# Create __init__.py files
touch src/vehicle_tools/__init__.py
touch src/vehicle_tools/gps/__init__.py
touch src/vehicle_tools/navigation/__init__.py
touch src/vehicle_tools/control/__init__.py
touch src/vehicle_tools/simulation/__init__.py
touch src/vehicle_tools/utils/__init__.py

echo -e "${GREEN}=== Python Environment Setup Complete! ===${NC}"
echo -e "${YELLOW}To activate the environment in the future, run:${NC}"
echo "  cd /opt/ws_navi"
echo "  ./activate_env.sh"
echo ""
echo -e "${YELLOW}To install new Python packages:${NC}"
echo "  uv pip install <package-name>"
echo ""
echo -e "${YELLOW}To update dependencies:${NC}"
echo "  uv pip install -e . --upgrade"
echo ""
echo -e "${YELLOW}Development commands:${NC}"
echo "  ruff check .                  # Check code quality"
echo "  ruff format .                 # Format all code"
echo "  pre-commit run --all-files    # Run all pre-commit hooks"

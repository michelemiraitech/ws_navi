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

# Create virtual environment with uv
echo -e "${YELLOW}Creating virtual environment with Python 3.10...${NC}"
uv venv --python 3.10

# Activate virtual environment
echo -e "${YELLOW}Activating virtual environment...${NC}"
source .venv/bin/activate

# Install dependencies
echo -e "${YELLOW}Installing Python dependencies...${NC}"
uv pip install -e .

# Install development dependencies
echo -e "${YELLOW}Installing development dependencies...${NC}"
uv pip install -e ".[dev]"

# Install simulation dependencies  
echo -e "${YELLOW}Installing simulation dependencies...${NC}"
uv pip install -e ".[simulation]"

# Create activation script
echo -e "${YELLOW}Creating activation script...${NC}"
cat > activate_env.sh << 'EOF'
#!/bin/bash

# ROS 2 Vehicle Navigation Environment Setup
# This script activates both ROS 2 and Python virtual environment

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Activating ROS 2 Vehicle Navigation Environment...${NC}"

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Activate Python virtual environment
source .venv/bin/activate

# Source workspace if built
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
    echo -e "${GREEN}ROS 2 workspace sourced successfully!${NC}"
else
    echo -e "${YELLOW}ROS 2 workspace not built yet. Run 'colcon build' first.${NC}"
fi

# Set Python path to include our packages
export PYTHONPATH="$PYTHONPATH:$(pwd)/src"

# Set environment variables
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/vehicle_simulation/models
export ROS_DOMAIN_ID=42

echo -e "${GREEN}Environment activated!${NC}"
echo -e "${YELLOW}Python virtual environment: $(which python)${NC}"
echo -e "${YELLOW}Available commands:${NC}"
echo "  uv pip list                    # List installed packages"
echo "  uv pip install <package>      # Install new package"
echo "  colcon build                  # Build ROS workspace"
echo "  pytest                        # Run Python tests"
echo "  ruff check src/               # Lint Python code"
echo "  ruff format src/              # Format Python code"
echo "  mypy src/                     # Type check Python code"
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

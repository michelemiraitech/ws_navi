#!/bin/bash

# Dagger Pipeline Development and Testing Script
# Usage: ./dev.sh <command> [options]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
ROS_DISTRO=${ROS_DISTRO:-jazzy}
PYTHON_VERSION=${PYTHON_VERSION:-3.10}
RUN_INTEGRATION=${RUN_INTEGRATION:-true}
RUN_LINTING=${RUN_LINTING:-true}

print_help() {
    echo -e "${BLUE}Dagger Pipeline Development Script${NC}"
    echo -e "${GREEN}🔄 Uses the same Dagger pipeline as GitHub Actions CI${NC}"
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  test      Run the complete CI pipeline locally (same as GitHub Actions)"
    echo "  lint      Run only linting checks"
    echo "  docs      Build documentation"
    echo "  check     Check if Dagger is properly installed"
    echo "  setup     Setup development environment"
    echo "  clean     Clean build artifacts and containers"
    echo ""
    echo "Options:"
    echo "  --ros-distro <distro>     ROS distribution (default: jazzy)"
    echo "  --python-version <ver>    Python version (default: 3.10)"
    echo "  --no-integration         Skip integration tests"
    echo "  --no-lint                Skip linting"
    echo ""
    echo -e "${YELLOW}GitHub Actions Integration:${NC}"
    echo "  • The GitHub Actions workflow (.github/workflows/ci.yml) uses these same commands"
    echo "  • Matrix testing with multiple ROS distros and Python versions"
    echo "  • Documentation auto-deploys to GitHub Pages"
    echo "  • Same containers and environment as local development"
    echo ""
    echo "Examples:"
    echo "  $0 test"
    echo "  $0 test --ros-distro humble --python-version 3.11"
    echo "  $0 lint"
    echo "  ROS_DISTRO=humble $0 test --no-integration"
}

check_dependencies() {
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    # Check for Dagger CLI
    if ! command -v dagger &> /dev/null; then
        echo -e "${RED}❌ Dagger CLI not found${NC}"
        echo "Install from: https://docs.dagger.io/install"
        exit 1
    else
        echo -e "${GREEN}✅ Dagger CLI found: $(dagger version)${NC}"
    fi
    
    # Check for Python
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}❌ Python 3 not found${NC}"
        exit 1
    else
        echo -e "${GREEN}✅ Python found: $(python3 --version)${NC}"
    fi
    
    # Check for Docker
    if ! command -v docker &> /dev/null; then
        echo -e "${YELLOW}⚠️  Docker not found - Dagger may not work properly${NC}"
    else
        echo -e "${GREEN}✅ Docker found${NC}"
    fi
}

setup_environment() {
    echo -e "${BLUE}Setting up development environment...${NC}"
    
    # Create Python virtual environment if it doesn't exist
    if [ ! -d ".venv" ]; then
        echo "Creating Python virtual environment..."
        python3 -m venv .venv
    fi
    
    # Activate virtual environment
    source .venv/bin/activate
    
    # Install development dependencies
    echo "Installing Python dependencies..."
    pip install --upgrade pip
    pip install dagger-io click
    
    # Make scripts executable
    chmod +x ci/run.sh
    chmod +x ci/dev.sh
    
    echo -e "${GREEN}✅ Development environment setup complete${NC}"
    echo "To activate the environment, run: source .venv/bin/activate"
}

run_pipeline() {
    local command=$1
    shift
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ros-distro)
                ROS_DISTRO="$2"
                shift 2
                ;;
            --python-version)
                PYTHON_VERSION="$2"
                shift 2
                ;;
            --no-integration)
                RUN_INTEGRATION="false"
                shift
                ;;
            --no-lint)
                RUN_LINTING="false"
                shift
                ;;
            *)
                echo -e "${RED}Unknown option: $1${NC}"
                exit 1
                ;;
        esac
    done
    
    echo -e "${BLUE}Running Dagger pipeline...${NC}"
    echo "ROS Distro: $ROS_DISTRO"
    echo "Python Version: $PYTHON_VERSION"
    echo "Integration Tests: $RUN_INTEGRATION"
    echo "Linting: $RUN_LINTING"
    echo ""
    
    case $command in
        test)
            dagger call build-and-test \
                --source=. \
                --ros-distro="$ROS_DISTRO" \
                --python-version="$PYTHON_VERSION" \
                --run-integration-tests="$RUN_INTEGRATION" \
                --run-linting="$RUN_LINTING"
            ;;
        lint)
            dagger call lint-only \
                --source=. \
                --python-version="$PYTHON_VERSION"
            ;;
        docs)
            echo "Building documentation..."
            dagger call build-docs --source=. export --path=./docs-output
            echo -e "${GREEN}✅ Documentation built in ./docs-output${NC}"
            ;;
        *)
            echo -e "${RED}Unknown command: $command${NC}"
            exit 1
            ;;
    esac
}

clean_artifacts() {
    echo -e "${BLUE}Cleaning build artifacts...${NC}"
    
    # Remove build directories
    rm -rf build/ install/ log/
    
    # Remove Python cache
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find . -type f -name "*.pyc" -delete 2>/dev/null || true
    
    # Remove documentation output
    rm -rf docs-output/
    
    # Clean Dagger cache (optional)
    read -p "Clean Dagger cache? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        dagger cache prune
    fi
    
    echo -e "${GREEN}✅ Cleanup complete${NC}"
}

# Main script logic
case "${1:-help}" in
    test|lint|docs)
        check_dependencies
        run_pipeline "$@"
        ;;
    check)
        check_dependencies
        echo -e "${GREEN}✅ All dependencies are available${NC}"
        ;;
    setup)
        check_dependencies
        setup_environment
        ;;
    clean)
        clean_artifacts
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        echo -e "${RED}Unknown command: ${1}${NC}"
        echo ""
        print_help
        exit 1
        ;;
esac

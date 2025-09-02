#!/bin/bash

# Dagger CI Pipeline Runner Script
# This script provides easy commands to run the Dagger CI pipeline

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

#!/bin/bash

# Dagger CI Pipeline Runner Script
# This script provides a unified interface to run the Dagger-based CI pipeline
# both locally and in CI environments (GitHub Actions)

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values - can be overridden by environment variables
ROS_DISTRO=${ROS_DISTRO:-jazzy}
PYTHON_VERSION=${PYTHON_VERSION:-3.10}
RUN_INTEGRATION=${RUN_INTEGRATION:-true}
RUN_LINTING=${RUN_LINTING:-true}
LINT_ONLY=${LINT_ONLY:-false}

print_help() {
    echo -e "${BLUE}Dagger CI Pipeline Runner${NC}"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --ros-distro <distro>     ROS distribution (default: jazzy)"
    echo "  --python-version <ver>    Python version (default: 3.10)"
    echo "  --no-integration         Skip integration tests"
    echo "  --no-lint                Skip linting"
    echo "  --lint-only              Run only linting checks"
    echo "  --help                   Show this help message"
    echo ""
    echo "Environment Variables:"
    echo "  ROS_DISTRO               ROS distribution"
    echo "  PYTHON_VERSION           Python version"
    echo "  RUN_INTEGRATION          Run integration tests (true/false)"
    echo "  RUN_LINTING              Run linting (true/false)"
    echo "  LINT_ONLY                Run only linting (true/false)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Run full pipeline with defaults"
    echo "  $0 --lint-only                       # Run only linting"
    echo "  $0 --ros-distro humble --no-integration"
    echo "  ROS_DISTRO=humble $0 --python-version 3.11"
    echo ""
    echo "GitHub Actions Integration:"
    echo "  This script uses the same Dagger pipeline as GitHub Actions CI."
    echo "  The CI workflow calls the same Dagger functions for consistency."
}

check_dependencies() {
    echo -e "${BLUE}Checking dependencies...${NC}"

    # Check for Dagger CLI
    if ! command -v dagger &> /dev/null; then
        echo -e "${RED}❌ Dagger CLI not found${NC}"
        echo "Install from: https://docs.dagger.io/install"
        echo ""
        echo "Quick install:"
        echo "curl -L https://dl.dagger.io/dagger/install.sh | sh"
        echo "sudo mv bin/dagger /usr/local/bin"
        exit 1
    else
        DAGGER_VERSION=$(dagger version 2>/dev/null | head -1 || echo "unknown")
        echo -e "${GREEN}✅ Dagger CLI found: ${DAGGER_VERSION}${NC}"
    fi

    # Check for Docker (Dagger backend)
    if ! command -v docker &> /dev/null; then
        echo -e "${YELLOW}⚠️  Docker not found - Dagger may not work properly${NC}"
        echo "Install Docker from: https://docs.docker.com/get-docker/"
    else
        if ! docker info &> /dev/null; then
            echo -e "${YELLOW}⚠️  Docker daemon not running${NC}"
            echo "Start Docker daemon and try again"
        else
            echo -e "${GREEN}✅ Docker is running${NC}"
        fi
    fi

    # Check if we're in the right directory
    if [ ! -f "dagger.toml" ]; then
        echo -e "${RED}❌ dagger.toml not found${NC}"
        echo "Please run this script from the project root directory"
        exit 1
    else
        echo -e "${GREEN}✅ Found dagger.toml - in correct directory${NC}"
    fi
}

print_usage() {
    echo -e "${BLUE}Dagger CI Pipeline for ROS 2 Vehicle Navigation${NC}"
    echo -e "${GREEN}🔄 This script uses the SAME Dagger pipeline as GitHub Actions CI${NC}"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build-and-test    Run complete CI pipeline (default)"
    echo "  lint              Run only linting checks"
    echo "  docs              Build documentation"
    echo "  install-dagger    Install Dagger CLI"
    echo ""
    echo "Environment Variables:"
    echo "  ROS_DISTRO               ROS distribution (default: jazzy)"
    echo "  PYTHON_VERSION           Python version (default: 3.10)"
    echo "  RUN_INTEGRATION_TESTS    Run integration tests (default: true)"
    echo "  RUN_LINTING              Run linting (default: true)"
    echo ""
    echo "Examples:"
    echo "  $0 build-and-test                    # Full pipeline (same as GitHub Actions)"
    echo "  ROS_DISTRO=humble $0 build-and-test # Test with ROS Humble"
    echo "  $0 lint                              # Quick linting only"
    echo "  $0 docs                              # Build documentation"
    echo ""
    echo -e "${YELLOW}GitHub Actions Integration:${NC}"
    echo "  • The .github/workflows/ci.yml file uses these same Dagger functions"
    echo "  • Matrix builds test multiple ROS distros (jazzy, humble)"
    echo "  • Same build environment and tests as your local runs"
    echo "  • Documentation deploys to GitHub Pages on main branch"
}

install_dagger() {
    echo -e "${YELLOW}Installing Dagger CLI...${NC}"

    if command -v dagger &> /dev/null; then
        echo -e "${GREEN}✓ Dagger CLI already installed${NC}"
        dagger version
        return 0
    fi

    # Install Dagger CLI
    curl -L https://dl.dagger.io/dagger/install.sh | BIN_DIR=/usr/local/bin sudo -E sh

    if command -v dagger &> /dev/null; then
        echo -e "${GREEN}✓ Dagger CLI installed successfully${NC}"
        dagger version
    else
        echo -e "${RED}✗ Failed to install Dagger CLI${NC}"
        exit 1
    fi
}

check_dagger() {
    if ! command -v dagger &> /dev/null; then
        echo -e "${YELLOW}Dagger CLI not found. Installing...${NC}"
        install_dagger
    fi
}

run_full_pipeline() {
    echo -e "${BLUE}🚀 Running Complete CI Pipeline${NC}"
    echo -e "${GREEN}📋 Same pipeline that runs in GitHub Actions CI${NC}"
    echo -e "${YELLOW}Configuration:${NC}"
    echo "  ROS Distribution: $ROS_DISTRO"
    echo "  Python Version: $PYTHON_VERSION"
    echo "  Integration Tests: $RUN_INTEGRATION_TESTS"
    echo "  Linting: $RUN_LINTING"
    echo ""

    check_dagger

    echo -e "${YELLOW}🔧 Executing Dagger pipeline...${NC}"
    dagger call build-and-test \
        --source . \
        --ros-distro "$ROS_DISTRO" \
        --python-version "$PYTHON_VERSION" \
        --run-integration-tests="$RUN_INTEGRATION_TESTS" \
        --run-linting="$RUN_LINTING"

    echo -e "${GREEN}✅ Local pipeline completed - matches GitHub Actions results${NC}"
}

run_lint_only() {
    echo -e "${BLUE}🔍 Running Linting Checks Only${NC}"
    echo -e "${YELLOW}Configuration:${NC}"
    echo "  Python Version: $PYTHON_VERSION"
    echo ""

    check_dagger

    dagger call lint-only \
        --source . \
        --python-version "$PYTHON_VERSION"
}

build_docs() {
    echo -e "${BLUE}📚 Building Documentation${NC}"

    check_dagger

    echo -e "${YELLOW}Building documentation...${NC}"
    dagger call build-docs --source . export --path ./docs-output

    if [ -d "./docs-output" ]; then
        echo -e "${GREEN}✓ Documentation built successfully${NC}"
        echo -e "${YELLOW}Output directory: ./docs-output${NC}"
        echo -e "${YELLOW}Open ./docs-output/index.html in your browser${NC}"
    else
        echo -e "${RED}✗ Documentation build failed${NC}"
        exit 1
    fi
}

# Main script logic
case "${1:-build-and-test}" in
    "build-and-test")
        run_full_pipeline
        ;;
    "lint")
        run_lint_only
        ;;
    "docs")
        build_docs
        ;;
    "install-dagger")
        install_dagger
        ;;
    "-h"|"--help"|"help")
        print_usage
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        echo ""
        print_usage
        exit 1
        ;;
esac

echo -e "${GREEN}🎉 Pipeline completed successfully!${NC}"
echo -e "${BLUE}💡 This local run used the same Dagger pipeline as GitHub Actions CI${NC}"

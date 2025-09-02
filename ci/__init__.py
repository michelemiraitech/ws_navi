"""Dagger configuration and utility functions."""

import os
from pathlib import Path


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).parent.parent


def get_ros_distro() -> str:
    """Get ROS distro from environment or default."""
    return os.getenv("ROS_DISTRO", "jazzy")


def get_python_version() -> str:
    """Get Python version from environment or default."""
    return os.getenv("PYTHON_VERSION", "3.10")


# Dagger module metadata
DAGGER_MODULE_NAME = "vehicle-navigation"
DAGGER_MODULE_DESCRIPTION = "CI/CD pipeline for ROS 2 Vehicle Navigation project"

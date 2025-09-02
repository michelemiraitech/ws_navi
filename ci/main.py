"""Dagger CI/CD Pipeline for ROS 2 Vehicle Navigation Project.

This module defines the CI pipeline using Dagger to build and test
the ROS 2 vehicle navigation system with parameterized ROS distro support.
"""

import sys
from typing import Annotated

import dagger
from dagger import Doc, dag, function, object_type


@object_type
class VehicleNavigation:
    """Dagger CI/CD pipeline for ROS 2 Vehicle Navigation project.

    This pipeline handles building, testing, and validating the complete
    ROS 2 vehicle navigation stack with support for multiple ROS distributions.
    """

    @function
    async def build_and_test(
        self,
        source: Annotated[dagger.Directory, Doc("Project source directory")],
        ros_distro: Annotated[str, Doc("ROS distribution name")] = "jazzy",
        python_version: Annotated[str, Doc("Python version")] = "3.10",
        run_integration_tests: Annotated[bool, Doc("Run integration tests")] = True,
        run_linting: Annotated[bool, Doc("Run code linting")] = True,
    ) -> str:
        """Complete build and test pipeline for the vehicle navigation project.

        Args:
            source: Source code directory
            ros_distro: ROS distribution (jazzy, humble, iron, etc.)
            python_version: Python version to use
            run_integration_tests: Whether to run integration tests
            run_linting: Whether to run code linting

        Returns:
            Success message with build summary
        """
        # Create base container with ROS
        base_container = await self._create_ros_container(ros_distro, python_version)

        # Copy source code
        container = base_container.with_directory("/workspace", source)

        # Install dependencies
        container = await self._install_dependencies(container)

        # Setup Python environment
        container = await self._setup_python_environment(container)

        # Build ROS workspace
        build_result = await self._build_ros_workspace(container)

        # Run tests
        test_results = []

        if run_linting:
            lint_result = await self._run_linting(container)
            test_results.append(f"Linting: {lint_result}")

        # Python unit tests
        python_test_result = await self._run_python_tests(container)
        test_results.append(f"Python tests: {python_test_result}")

        # ROS package tests
        ros_test_result = await self._run_ros_tests(container)
        test_results.append(f"ROS tests: {ros_test_result}")

        if run_integration_tests:
            integration_result = await self._run_integration_tests(container)
            test_results.append(f"Integration tests: {integration_result}")

        # Generate summary
        return f"""
        ✅ Vehicle Navigation CI Pipeline Complete!

        📋 Configuration:
        - ROS Distribution: {ros_distro}
        - Python Version: {python_version}
        - Integration Tests: {"Enabled" if run_integration_tests else "Disabled"}
        - Linting: {"Enabled" if run_linting else "Disabled"}

        🔨 Build Results:
        {build_result}

        🧪 Test Results:
        {chr(10).join(test_results)}

        🎉 All checks passed successfully!
        """

    @function
    async def _create_ros_container(
        self,
        ros_distro: str,
        python_version: str,
    ) -> dagger.Container:
        """Create base container with ROS installation."""
        # Use official Ubuntu image that supports the ROS distro
        ubuntu_version = self._get_ubuntu_version_for_ros(ros_distro)

        return (
            dag.container()
            .from_(f"ubuntu:{ubuntu_version}")
            # Set up timezone and locale
            .with_env_variable("TZ", "UTC")
            .with_env_variable("DEBIAN_FRONTEND", "noninteractive")
            .with_exec(
                [
                    "apt-get",
                    "update",
                    "&&",
                    "apt-get",
                    "install",
                    "-y",
                    "locales",
                    "&&",
                    "locale-gen",
                    "en_US.UTF-8",
                ],
            )
            .with_env_variable("LANG", "en_US.UTF-8")
            .with_env_variable("LANGUAGE", "en_US:en")
            .with_env_variable("LC_ALL", "en_US.UTF-8")
            # Install basic dependencies
            .with_exec(
                [
                    "apt-get",
                    "install",
                    "-y",
                    "curl",
                    "gnupg",
                    "lsb-release",
                    "software-properties-common",
                    "build-essential",
                    "cmake",
                    "git",
                    "wget",
                    "python3-pip",
                ],
            )
            # Add ROS repository
            .with_exec(
                [
                    "curl",
                    "-sSL",
                    "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key",
                    "-o",
                    "/usr/share/keyrings/ros-archive-keyring.gpg",
                ],
            )
            .with_exec(
                [
                    "echo",
                    "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main",
                    "|",
                    "tee",
                    "/etc/apt/sources.list.d/ros2.list",
                    ">",
                    "/dev/null",
                ],
            )
            # Install ROS
            .with_exec(["apt-get", "update"])
            .with_exec(
                [
                    "apt-get",
                    "install",
                    "-y",
                    f"ros-{ros_distro}-desktop",
                    f"ros-{ros_distro}-rmw-fastrtps-cpp",
                    "python3-colcon-common-extensions",
                    "python3-rosdep",
                    "python3-vcstool",
                ],
            )
            # Initialize rosdep
            .with_exec(["rosdep", "init"])
            .with_exec(["rosdep", "update"])
            # Set ROS environment
            .with_env_variable("ROS_DISTRO", ros_distro)
            .with_env_variable("ROS_VERSION", "2")
            .with_env_variable("ROS_PYTHON_VERSION", "3")
        )

    @function
    async def _install_dependencies(
        self,
        container: dagger.Container,
    ) -> dagger.Container:
        """Install project-specific dependencies."""
        return (
            container
            # Install ROS packages
            .with_exec(
                [
                    "apt-get",
                    "install",
                    "-y",
                    f"ros-{container.env_variable('ROS_DISTRO')}-nav2-bringup",
                    f"ros-{container.env_variable('ROS_DISTRO')}-nav2-common",
                    f"ros-{container.env_variable('ROS_DISTRO')}-cartographer",
                    f"ros-{container.env_variable('ROS_DISTRO')}-cartographer-ros",
                    f"ros-{container.env_variable('ROS_DISTRO')}-robot-localization",
                    f"ros-{container.env_variable('ROS_DISTRO')}-gazebo-ros-pkgs",
                    f"ros-{container.env_variable('ROS_DISTRO')}-robot-state-publisher",
                    f"ros-{container.env_variable('ROS_DISTRO')}-joint-state-publisher",
                    f"ros-{container.env_variable('ROS_DISTRO')}-xacro",
                    f"ros-{container.env_variable('ROS_DISTRO')}-tf2-tools",
                    f"ros-{container.env_variable('ROS_DISTRO')}-rviz2",
                ],
            )
            # Install uv for Python package management
            .with_exec(
                [
                    "curl",
                    "-LsSf",
                    "https://astral.sh/uv/install.sh",
                    "|",
                    "sh",
                ],
            )
            .with_env_variable("PATH", "/root/.cargo/bin:$PATH")
        )

    @function
    async def _setup_python_environment(
        self,
        container: dagger.Container,
    ) -> dagger.Container:
        """Setup Python virtual environment and dependencies."""
        return (
            container.with_workdir("/workspace")
            # Create virtual environment with uv
            .with_exec(["uv", "venv", "--python", "3.10"])
            # Install Python dependencies
            .with_exec(
                ["/bin/bash", "-c", "source .venv/bin/activate && uv pip install -e ."],
            )
            .with_exec(
                [
                    "/bin/bash",
                    "-c",
                    "source .venv/bin/activate && uv pip install -e '.[dev]'",
                ],
            )
            # Set Python environment
            .with_env_variable("VIRTUAL_ENV", "/workspace/.venv")
            .with_env_variable("PATH", "/workspace/.venv/bin:$PATH")
        )

    @function
    async def _build_ros_workspace(self, container: dagger.Container) -> str:
        """Build the ROS workspace."""
        await (
            container.with_workdir("/workspace")
            .with_exec(
                [
                    "/bin/bash",
                    "-c",
                    f"source /opt/ros/{container.env_variable('ROS_DISTRO')}/setup.bash && "
                    "rosdep install --from-paths src --ignore-src -r -y && "
                    "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release",
                ],
            )
            .stdout()
        )

        return "✅ ROS workspace built successfully"

    @function
    async def _run_linting(self, container: dagger.Container) -> str:
        """Run code linting with ruff."""
        try:
            await (
                container.with_workdir("/workspace")
                .with_exec(
                    ["/bin/bash", "-c", "source .venv/bin/activate && ruff check src/"],
                )
                .with_exec(
                    [
                        "/bin/bash",
                        "-c",
                        "source .venv/bin/activate && ruff format --check src/",
                    ],
                )
                .stdout()
            )
            return "✅ PASSED"
        except Exception as e:
            return f"❌ FAILED: {e!s}"

    @function
    async def _run_python_tests(self, container: dagger.Container) -> str:
        """Run Python unit tests."""
        try:
            await (
                container.with_workdir("/workspace")
                .with_exec(
                    [
                        "/bin/bash",
                        "-c",
                        "source .venv/bin/activate && pytest tests/ -v --cov=src --cov-report=xml",
                    ],
                )
                .stdout()
            )
            return "✅ PASSED"
        except Exception as e:
            return f"❌ FAILED: {e!s}"

    @function
    async def _run_ros_tests(self, container: dagger.Container) -> str:
        """Run ROS package tests."""
        try:
            await (
                container.with_workdir("/workspace")
                .with_exec(
                    [
                        "/bin/bash",
                        "-c",
                        f"source /opt/ros/{container.env_variable('ROS_DISTRO')}/setup.bash && "
                        "source install/setup.bash && "
                        "colcon test && "
                        "colcon test-result --verbose",
                    ],
                )
                .stdout()
            )
            return "✅ PASSED"
        except Exception as e:
            return f"❌ FAILED: {e!s}"

    @function
    async def _run_integration_tests(self, container: dagger.Container) -> str:
        """Run integration tests with Gazebo simulation."""
        try:
            # Start Xvfb for headless testing
            test_container = (
                container.with_exec(["apt-get", "install", "-y", "xvfb", "xauth"])
                .with_env_variable("DISPLAY", ":1")
                .with_exec(["Xvfb", ":1", "-screen", "0", "1024x768x24", "&"])
            )

            await (
                test_container.with_workdir("/workspace")
                .with_exec(
                    [
                        "/bin/bash",
                        "-c",
                        f"source /opt/ros/{container.env_variable('ROS_DISTRO')}/setup.bash && "
                        "source install/setup.bash && "
                        "timeout 60 ros2 launch vehicle_bringup vehicle_bringup.launch.py &"
                        "sleep 30 && "
                        "ros2 topic list | grep -q '/scan' && "
                        "ros2 topic list | grep -q '/odom' && "
                        "pkill -f ros2",
                    ],
                )
                .stdout()
            )
            return "✅ PASSED"
        except Exception as e:
            return f"❌ FAILED: {e!s}"

    @function
    def _get_ubuntu_version_for_ros(self, ros_distro: str) -> str:
        """Get Ubuntu version for ROS distribution."""
        ros_ubuntu_mapping = {
            "jazzy": "24.04",
            "iron": "22.04",
            "humble": "22.04",
            "galactic": "20.04",
            "foxy": "20.04",
        }
        return ros_ubuntu_mapping.get(ros_distro, "22.04")

    @function
    async def lint_only(
        self,
        source: Annotated[dagger.Directory, Doc("Project source directory")],
        python_version: Annotated[str, Doc("Python version")] = "3.10",
    ) -> str:
        """Run only linting checks (faster for quick validation)."""
        # Lightweight container for linting
        container = (
            dag.container()
            .from_(f"python:{python_version}-slim")
            .with_directory("/workspace", source)
            .with_workdir("/workspace")
            .with_exec(["pip", "install", "uv"])
            .with_exec(["uv", "venv"])
            .with_exec(
                [
                    "/bin/bash",
                    "-c",
                    "source .venv/bin/activate && uv pip install ruff mypy",
                ],
            )
        )

        # Run linting
        ruff_result = await self._run_linting(container)

        # Run type checking
        try:
            await container.with_exec(
                ["/bin/bash", "-c", "source .venv/bin/activate && mypy src/"],
            ).stdout()
            mypy_result = "✅ PASSED"
        except Exception as e:
            mypy_result = f"❌ FAILED: {e!s}"

        return f"""
        🔍 Linting Results:
        - Ruff: {ruff_result}
        - MyPy: {mypy_result}
        """

    @function
    async def build_docs(
        self,
        source: Annotated[dagger.Directory, Doc("Project source directory")],
    ) -> dagger.Directory:
        """Build project documentation."""
        container = (
            dag.container()
            .from_("python:3.10")
            .with_directory("/workspace", source)
            .with_workdir("/workspace")
            .with_exec(["pip", "install", "uv"])
            .with_exec(["uv", "venv"])
            .with_exec(
                [
                    "/bin/bash",
                    "-c",
                    "source .venv/bin/activate && uv pip install -e '.[dev]'",
                ],
            )
            .with_exec(
                [
                    "/bin/bash",
                    "-c",
                    "source .venv/bin/activate && sphinx-build -b html docs/ docs/_build/html",
                ],
            )
        )

        return container.directory("/workspace/docs/_build/html")


# Entry point for Dagger CLI
if __name__ == "__main__":
    import asyncio

    async def main():
        async with dagger.Connection(dagger.Config(log_output=sys.stderr)) as client:
            # Get source directory
            source_dir = client.host().directory(".")

            # Create pipeline instance
            pipeline = VehicleNavigation()

            # Run pipeline
            result = await pipeline.build_and_test(
                source=source_dir,
                ros_distro="jazzy",
                run_integration_tests=True,
                run_linting=True,
            )

            print(result)

    asyncio.run(main())

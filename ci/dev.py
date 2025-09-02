"""Local development utilities for Dagger CI pipeline."""

import asyncio
import sys
from pathlib import Path

import click

# Add the ci directory to Python path for imports
sys.path.insert(0, str(Path(__file__).parent))

try:
    import dagger
    from main import VehicleNavigation

    DAGGER_AVAILABLE = True
except ImportError:
    DAGGER_AVAILABLE = False


@click.group()
def cli():
    """Local development utilities for the Dagger CI pipeline."""
    if not DAGGER_AVAILABLE:
        click.echo("❌ Dagger not available. Install with: pip install dagger-io")
        sys.exit(1)


@cli.command()
@click.option("--ros-distro", default="jazzy", help="ROS distribution")
@click.option("--python-version", default="3.10", help="Python version")
@click.option(
    "--integration/--no-integration",
    default=True,
    help="Run integration tests",
)
@click.option("--lint/--no-lint", default=True, help="Run linting")
def test(ros_distro, python_version, integration, lint):
    """Run the complete CI pipeline locally."""

    async def run_pipeline():
        async with dagger.Connection(dagger.Config(log_output=sys.stderr)) as client:
            # Get source directory
            source_dir = client.host().directory(".")

            # Create pipeline instance
            pipeline = VehicleNavigation()

            # Run pipeline
            result = await pipeline.build_and_test(
                source=source_dir,
                ros_distro=ros_distro,
                python_version=python_version,
                run_integration_tests=integration,
                run_linting=lint,
            )

            click.echo(result)

    asyncio.run(run_pipeline())


@cli.command()
@click.option("--python-version", default="3.10", help="Python version")
def lint(python_version):
    """Run only linting checks."""

    async def run_lint():
        async with dagger.Connection(dagger.Config(log_output=sys.stderr)) as client:
            source_dir = client.host().directory(".")
            pipeline = VehicleNavigation()

            result = await pipeline.lint_only(
                source=source_dir,
                python_version=python_version,
            )

            click.echo(result)

    asyncio.run(run_lint())


@cli.command()
def docs():
    """Build documentation."""

    async def build_docs():
        async with dagger.Connection(dagger.Config(log_output=sys.stderr)) as client:
            source_dir = client.host().directory(".")
            pipeline = VehicleNavigation()

            # Build docs and export to local directory
            docs_dir = await pipeline.build_docs(source=source_dir)
            await docs_dir.export("./docs-output")

            click.echo("✅ Documentation built in ./docs-output")

    asyncio.run(build_docs())


@cli.command()
def check():
    """Check if Dagger and dependencies are properly installed."""
    try:
        import dagger

        click.echo("✅ Dagger SDK installed")
    except ImportError:
        click.echo("❌ Dagger SDK not found. Install with: pip install dagger-io")
        return

    # Check if Dagger CLI is available
    import subprocess

    try:
        result = subprocess.run(["dagger", "version"], capture_output=True, text=True)
        if result.returncode == 0:
            click.echo(f"✅ Dagger CLI: {result.stdout.strip()}")
        else:
            click.echo("❌ Dagger CLI not working properly")
    except FileNotFoundError:
        click.echo("❌ Dagger CLI not found. Install from https://dagger.io")


if __name__ == "__main__":
    cli()

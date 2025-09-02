"""
Setup utilities for the vehicle navigation system.
"""

import click
import os
import subprocess
import sys
from pathlib import Path
from rich.console import Console
from rich.panel import Panel
from rich.progress import Progress


console = Console()


def check_ros_installation():
    """Check if ROS 2 is properly installed."""
    try:
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            console.print(f"✓ ROS 2 found: {result.stdout.strip()}", style="green")
            return True
    except FileNotFoundError:
        pass
    
    console.print("✗ ROS 2 not found", style="red")
    return False


def check_dependencies():
    """Check if required dependencies are installed."""
    required_packages = [
        'numpy', 'scipy', 'matplotlib', 'pandas', 
        'utm', 'pyproj', 'geopy', 'pyyaml'
    ]
    
    missing = []
    for package in required_packages:
        try:
            __import__(package)
            console.print(f"✓ {package}", style="green")
        except ImportError:
            console.print(f"✗ {package}", style="red")
            missing.append(package)
    
    return missing


def setup_workspace():
    """Set up the ROS workspace."""
    workspace_dir = Path("/opt/ws_navi")
    
    if not workspace_dir.exists():
        console.print("✗ Workspace directory not found", style="red")
        return False
    
    # Check if workspace is built
    if (workspace_dir / "install").exists():
        console.print("✓ Workspace appears to be built", style="green")
    else:
        console.print("⚠ Workspace not built yet", style="yellow")
        console.print("Run 'colcon build' in the workspace directory")
    
    return True


def create_desktop_entry():
    """Create a desktop entry for easy access."""
    desktop_content = """[Desktop Entry]
Name=Vehicle Navigation
Comment=ROS 2 Vehicle Navigation System
Exec=gnome-terminal --working-directory=/opt/ws_navi -- bash -c "./activate_env.sh; bash"
Icon=applications-science
Terminal=false
Type=Application
Categories=Development;Science;
"""
    
    desktop_dir = Path.home() / ".local/share/applications"
    desktop_dir.mkdir(parents=True, exist_ok=True)
    
    desktop_file = desktop_dir / "vehicle-navigation.desktop"
    with open(desktop_file, 'w') as f:
        f.write(desktop_content)
    
    # Make executable
    os.chmod(desktop_file, 0o755)
    
    console.print(f"✓ Desktop entry created: {desktop_file}", style="green")


@click.group()
def cli():
    """Vehicle Navigation System Setup Tools."""
    pass


@cli.command()
def check():
    """Check system requirements and dependencies."""
    console.print(Panel.fit("System Requirements Check", style="bold blue"))
    
    # Check ROS
    ros_ok = check_ros_installation()
    
    # Check Python dependencies
    console.print("\nPython Dependencies:", style="bold")
    missing = check_dependencies()
    
    # Check workspace
    console.print("\nWorkspace Status:", style="bold")
    workspace_ok = setup_workspace()
    
    # Summary
    console.print("\nSummary:", style="bold")
    if ros_ok and not missing and workspace_ok:
        console.print("✓ All checks passed!", style="bold green")
    else:
        console.print("⚠ Some issues found. See above for details.", style="bold yellow")


@cli.command()
def install():
    """Install missing dependencies."""
    console.print(Panel.fit("Installing Dependencies", style="bold blue"))
    
    missing = check_dependencies()
    if missing:
        console.print(f"Installing: {', '.join(missing)}")
        with Progress() as progress:
            task = progress.add_task("Installing...", total=len(missing))
            for package in missing:
                subprocess.run([sys.executable, '-m', 'pip', 'install', package])
                progress.advance(task)
        console.print("✓ Dependencies installed", style="green")
    else:
        console.print("✓ All dependencies already installed", style="green")


@cli.command()
def desktop():
    """Create desktop entry for easy access."""
    create_desktop_entry()


@cli.command()
def build():
    """Build the ROS workspace."""
    workspace_dir = Path("/opt/ws_navi")
    
    if not workspace_dir.exists():
        console.print("✗ Workspace directory not found", style="red")
        return
    
    console.print("Building ROS workspace...", style="blue")
    
    # Change to workspace directory and build
    os.chdir(workspace_dir)
    result = subprocess.run(['colcon', 'build'], capture_output=True, text=True)
    
    if result.returncode == 0:
        console.print("✓ Workspace built successfully", style="green")
    else:
        console.print("✗ Build failed", style="red")
        console.print(result.stderr)


def main():
    """Entry point for the setup utility."""
    cli()


if __name__ == '__main__':
    main()

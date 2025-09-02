"""Navigation Analysis Tools for Vehicle Navigation System.

This module provides tools for analyzing navigation performance,
path planning, and system diagnostics.
"""

from pathlib import Path as PathLib

import click
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node


class NavigationAnalyzer(Node):
    """ROS 2 node for analyzing navigation performance and generating reports."""

    def __init__(self) -> None:
        super().__init__("navigation_analyzer")

        # Data storage
        self.global_paths: list[Path] = []
        self.local_paths: list[Path] = []
        self.cmd_vel_history: list[tuple[float, Twist]] = []
        self.pose_history: list[tuple[float, PoseStamped]] = []

        # Subscribers
        self.global_path_sub = self.create_subscription(
            Path,
            "/plan",
            self.global_path_callback,
            10,
        )
        self.local_path_sub = self.create_subscription(
            Path,
            "/local_plan",
            self.local_path_callback,
            10,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/current_pose",
            self.pose_callback,
            10,
        )

        self.get_logger().info("Navigation analyzer initialized")

    def global_path_callback(self, msg: Path) -> None:
        """Store global path for analysis."""
        self.global_paths.append(msg)

    def local_path_callback(self, msg: Path) -> None:
        """Store local path for analysis."""
        self.local_paths.append(msg)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Store velocity commands for analysis."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.cmd_vel_history.append((timestamp, msg))

        # Keep only last 1000 entries
        if len(self.cmd_vel_history) > 1000:
            self.cmd_vel_history.pop(0)

    def pose_callback(self, msg: PoseStamped) -> None:
        """Store pose history for analysis."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.pose_history.append((timestamp, msg))

        # Keep only last 1000 entries
        if len(self.pose_history) > 1000:
            self.pose_history.pop(0)

    def analyze_path_smoothness(self, path: Path) -> float:
        """Analyze path smoothness by calculating curvature variations.

        Returns:
            Average curvature variation (lower is smoother)
        """
        if len(path.poses) < 3:
            return 0.0

        curvatures = []
        for i in range(1, len(path.poses) - 1):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            p3 = path.poses[i + 1].pose.position

            # Calculate curvature using three points
            dx1 = p2.x - p1.x
            dy1 = p2.y - p1.y
            dx2 = p3.x - p2.x
            dy2 = p3.y - p2.y

            # Avoid division by zero
            d1 = np.sqrt(dx1**2 + dy1**2)
            d2 = np.sqrt(dx2**2 + dy2**2)

            if d1 > 1e-6 and d2 > 1e-6:
                angle_change = np.arctan2(dy2, dx2) - np.arctan2(dy1, dx1)
                curvature = angle_change / ((d1 + d2) / 2)
                curvatures.append(abs(curvature))

        return np.mean(curvatures) if curvatures else 0.0

    def analyze_velocity_profile(self) -> dict:
        """Analyze velocity command history.

        Returns:
            Dictionary with velocity statistics
        """
        if not self.cmd_vel_history:
            return {}

        linear_vels = [cmd.linear.x for _, cmd in self.cmd_vel_history]
        angular_vels = [cmd.angular.z for _, cmd in self.cmd_vel_history]

        return {
            "avg_linear_vel": np.mean(linear_vels),
            "max_linear_vel": np.max(linear_vels),
            "avg_angular_vel": np.mean(np.abs(angular_vels)),
            "max_angular_vel": np.max(np.abs(angular_vels)),
            "linear_vel_std": np.std(linear_vels),
            "angular_vel_std": np.std(angular_vels),
        }

    def generate_report(self, output_dir: str = "./nav_analysis") -> None:
        """Generate comprehensive navigation analysis report."""
        output_path = PathLib(output_dir)
        output_path.mkdir(exist_ok=True)

        # Analyze paths
        if self.global_paths:
            latest_global = self.global_paths[-1]
            global_smoothness = self.analyze_path_smoothness(latest_global)
        else:
            global_smoothness = 0.0

        # Analyze velocities
        vel_stats = self.analyze_velocity_profile()

        # Create report
        report = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "path_analysis": {
                "global_path_smoothness": global_smoothness,
                "num_global_paths": len(self.global_paths),
                "num_local_paths": len(self.local_paths),
            },
            "velocity_analysis": vel_stats,
            "pose_tracking": {
                "num_poses_recorded": len(self.pose_history),
            },
        }

        # Save report
        with open(output_path / "navigation_report.yaml", "w") as f:
            yaml.dump(report, f, default_flow_style=False)

        self.get_logger().info(f"Navigation report saved to {output_path}")


def plot_path_analysis(path_file: str, output_file: str = "path_analysis.png"):
    """Create visualization plots for path analysis.

    Args:
        path_file: Path to saved path data
        output_file: Output plot filename
    """
    # This is a placeholder for path visualization
    # In practice, you'd load path data and create plots
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Placeholder plots
    axes[0, 0].set_title("Global Path")
    axes[0, 1].set_title("Local Path Tracking")
    axes[1, 0].set_title("Velocity Profile")
    axes[1, 1].set_title("Navigation Performance")

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    plt.close()


@click.command()
@click.option("--analyze", is_flag=True, help="Run navigation analysis")
@click.option("--plot", type=str, help="Generate plots from data file")
@click.option("--output", default="./analysis", help="Output directory")
def main(analyze, plot, output):
    """Navigation analysis command-line interface."""
    if analyze:
        rclpy.init()
        analyzer = NavigationAnalyzer()

        try:
            # Run for a specified time or until interrupted
            rclpy.spin(analyzer)
        except KeyboardInterrupt:
            analyzer.generate_report(output)
        finally:
            analyzer.destroy_node()
            rclpy.shutdown()

    elif plot:
        plot_path_analysis(plot, f"{output}/path_analysis.png")
        click.echo(f"Plots saved to {output}/")

    else:
        click.echo("Use --analyze to run analysis or --plot <file> to generate plots")


if __name__ == "__main__":
    main()

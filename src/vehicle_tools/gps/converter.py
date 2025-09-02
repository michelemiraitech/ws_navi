"""GPS Utility Functions for Vehicle Navigation.

This module provides utilities for GPS coordinate conversion and processing
for the ROS 2 vehicle navigation system.
"""


import click
import numpy as np
import utm
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSToUTMConverter(Node):
    """ROS 2 node for converting GPS coordinates to UTM for local navigation.

    This node subscribes to GPS fix messages and publishes UTM coordinates
    that can be used with the robot_localization EKF filter.
    """

    def __init__(self) -> None:
        super().__init__("gps_utm_converter")

        # Publishers and subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "fix",
            self.gps_callback,
            10,
        )

        self.utm_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "gps/utm",
            10,
        )

        # UTM reference point (set on first GPS fix)
        self.utm_origin: tuple[float, float, int, str] | None = None
        self.origin_lat_lon: tuple[float, float] | None = None

        self.get_logger().info("GPS to UTM converter initialized")

    def gps_callback(self, msg: NavSatFix) -> None:
        """Process GPS fix message and convert to local UTM coordinates."""
        if msg.status.status < 0:  # No GPS fix
            return

        lat, lon = msg.latitude, msg.longitude

        # Convert to UTM
        easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)

        # Set origin on first valid GPS reading
        if self.utm_origin is None:
            self.utm_origin = (easting, northing, zone_num, zone_letter)
            self.origin_lat_lon = (lat, lon)
            self.get_logger().info(
                f"UTM origin set: E={easting:.2f}, N={northing:.2f}, " f"Zone={zone_num}{zone_letter}",
            )

        # Calculate local coordinates relative to origin
        local_x = easting - self.utm_origin[0]
        local_y = northing - self.utm_origin[1]

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "map"  # Global frame

        pose_msg.pose.pose.position.x = local_x
        pose_msg.pose.pose.position.y = local_y
        pose_msg.pose.pose.position.z = msg.altitude

        # Set covariance based on GPS accuracy
        # GPS typically has ~3m accuracy
        pos_var = max(3.0, msg.position_covariance[0]) ** 2
        pose_msg.pose.covariance[0] = pos_var  # x variance
        pose_msg.pose.covariance[7] = pos_var  # y variance
        pose_msg.pose.covariance[14] = pos_var * 2  # z variance (less accurate)

        self.utm_pub.publish(pose_msg)

    @staticmethod
    def latlon_to_utm(lat: float, lon: float) -> tuple[float, float, int, str]:
        """Convert latitude/longitude to UTM coordinates."""
        return utm.from_latlon(lat, lon)

    @staticmethod
    def utm_to_latlon(
        easting: float,
        northing: float,
        zone_num: int,
        zone_letter: str,
    ) -> tuple[float, float]:
        """Convert UTM coordinates to latitude/longitude."""
        return utm.to_latlon(easting, northing, zone_num, zone_letter)


def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two GPS coordinates using Haversine formula.

    Args:
        lat1, lon1: First point coordinates
        lat2, lon2: Second point coordinates

    Returns:
        Distance in meters
    """
    # Convert to radians
    lat1_rad = np.radians(lat1)
    lon1_rad = np.radians(lon1)
    lat2_rad = np.radians(lat2)
    lon2_rad = np.radians(lon2)

    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = np.sin(dlat / 2) ** 2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon / 2) ** 2
    c = 2 * np.arcsin(np.sqrt(a))

    # Earth radius in meters
    R = 6371000
    return R * c


@click.command()
@click.option("--lat", type=float, help="Latitude")
@click.option("--lon", type=float, help="Longitude")
@click.option("--easting", type=float, help="UTM Easting")
@click.option("--northing", type=float, help="UTM Northing")
@click.option("--zone-num", type=int, help="UTM Zone Number")
@click.option("--zone-letter", type=str, help="UTM Zone Letter")
def main(lat, lon, easting, northing, zone_num, zone_letter):
    """GPS coordinate conversion utility."""
    if lat is not None and lon is not None:
        # Convert lat/lon to UTM
        e, n, znum, zletter = GPSToUTMConverter.latlon_to_utm(lat, lon)
        click.echo(f"UTM: E={e:.2f}, N={n:.2f}, Zone={znum}{zletter}")
    elif all(x is not None for x in [easting, northing, zone_num, zone_letter]):
        # Convert UTM to lat/lon
        lat_out, lon_out = GPSToUTMConverter.utm_to_latlon(
            easting,
            northing,
            zone_num,
            zone_letter,
        )
        click.echo(f"Lat/Lon: {lat_out:.8f}, {lon_out:.8f}")
    else:
        click.echo("Please provide either --lat/--lon or UTM coordinates")


if __name__ == "__main__":
    main()

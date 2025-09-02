# Test Configuration for Vehicle Navigation
pytest_plugins = ["pytest_rclpy"]

# Test data directory
TEST_DATA_DIR = "tests/data"

# Mock ROS messages for testing without full ROS environment
MOCK_GPS_FIX = {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 50.0,
    "status": {"status": 0},  # GPS_FIX
}

MOCK_IMU_DATA = {
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
    "linear_acceleration": {"x": 0.1, "y": 0.0, "z": 9.81},
}

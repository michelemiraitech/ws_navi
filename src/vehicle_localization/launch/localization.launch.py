from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get config file
    ekf_config = PathJoinSubstitution([
        FindPackageShare("vehicle_localization"),
        "config",
        "ekf.yaml"
    ])

    # GPS to UTM conversion node (placeholder - you'd implement this)
    gps_utm_node = Node(
        package="vehicle_localization",
        executable="gps_utm_converter",
        name="gps_utm_converter",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("gps_input", "fix"),
            ("utm_output", "gps/utm")
        ]
    )

    # EKF node for sensor fusion
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[("odometry/filtered", "odometry/global")]
    )

    nodes = [
        # gps_utm_node,  # Uncomment when you implement the GPS UTM converter
        ekf_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

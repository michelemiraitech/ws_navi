from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam",
            default_value="false",
            description="Whether to run SLAM",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="test_world.world",
            description="World file name",
        ),
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")

    # Include robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_description"),
                    "launch",
                    "robot_state_publisher.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Include Gazebo simulation
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_simulation"),
                    "launch",
                    "gazebo.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "world": world,
        }.items(),
    )

    # Include localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_localization"),
                    "launch",
                    "localization.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Cartographer SLAM
    cartographer_config_dir = PathJoinSubstitution(
        [
            FindPackageShare("vehicle_bringup"),
            "config",
        ]
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            "cartographer.lua",
        ],
        condition=IfCondition(slam),
    )

    # Occupancy grid node for cartographer
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", "0.05"],
        condition=IfCondition(slam),
    )

    # Include navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_navigation"),
                    "launch",
                    "navigation.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": slam,
        }.items(),
    )

    nodes = [
        robot_state_publisher,
        gazebo_simulation,
        localization,
        cartographer_node,
        occupancy_grid_node,
        navigation,
    ]

    return LaunchDescription(declared_arguments + nodes)

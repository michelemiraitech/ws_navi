from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty_world.world",
            description="World file name",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "x_pose",
            default_value="0.0",
            description="x position",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "y_pose",
            default_value="0.0",
            description="y position",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "z_pose",
            default_value="0.0",
            description="z position",
        ),
    )

    # Initialize Arguments
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"],
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("vehicle_simulation"), "worlds", world],
            ),
        }.items(),
    )

    # Robot description
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_description"),
                    "urdf",
                    "vehicle_gazebo.urdf.xacro",
                ],
            ),
        ],
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "vehicle",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
        output="screen",
    )

    nodes = [
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ]

    return LaunchDescription(declared_arguments + nodes)

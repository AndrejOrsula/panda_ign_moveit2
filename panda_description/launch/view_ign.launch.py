#!/usr/bin/env -S ros2 launch
"""Visualisation of SDF model for panda in Ignition Gazebo. Note that the generated model://panda/model.sdf descriptor is used."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from typing import List


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_ign_gazebo"),
                        "launch",
                        "ign_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments=[("ign_args", [world, " -v ", ign_verbosity])],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # ros_ign_gazebo_create
        Node(
            package="ros_ign_gazebo",
            executable="create",
            output="log",
            arguments=["-file", model, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World and model for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]

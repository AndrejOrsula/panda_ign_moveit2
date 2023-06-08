#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 and executing motions using ROS 2 controllers within Gazebo"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    sdf_model_filepath = LaunchConfiguration("sdf_model_filepath")
    world = LaunchConfiguration("world")
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    collision_arm = LaunchConfiguration("collision_arm")
    collision_gripper = LaunchConfiguration("collision_gripper")
    ros2_control_command_interface = LaunchConfiguration(
        "ros2_control_command_interface"
    )
    gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of processes to be executed
    # xacro2sdf
    xacro2sdf = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindExecutable(name="ros2")]),
            "run",
            description_package,
            "xacro2sdf.bash",
            ["name:=", name],
            ["prefix:=", prefix],
            ["collision_arm:=", collision_arm],
            ["collision_gripper:=", collision_gripper],
            ["ros2_control:=", "true"],
            ["ros2_control_plugin:=", "gz"],
            ["ros2_control_command_interface:=", ros2_control_command_interface],
            ["gazebo_preserve_fixed_joint:=", gazebo_preserve_fixed_joint],
        ],
        shell=True,
    )
    processes = [xacro2sdf]

    # List of included launch descriptions
    launch_descriptions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=xacro2sdf,
                on_exit=[
                    # Launch Gazebo
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("ros_gz_sim"),
                                    "launch",
                                    "gz_sim.launch.py",
                                ]
                            )
                        ),
                        launch_arguments=[
                            ("gz_args", [world, " -r -v ", ign_verbosity])
                        ],
                    ),
                    # Launch move_group of MoveIt 2
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("panda_moveit_config"),
                                    "launch",
                                    "move_group.launch.py",
                                ]
                            )
                        ),
                        launch_arguments=[
                            ("name:=", name),
                            ("prefix:=", prefix),
                            ("collision_arm", collision_arm),
                            ("collision_gripper", collision_gripper),
                            ("ros2_control", "true"),
                            ("ros2_control_plugin", "gz"),
                            ("ros2_control_interface", ros2_control_command_interface),
                            (
                                "gazebo_preserve_fixed_joint",
                                gazebo_preserve_fixed_joint,
                            ),
                            ("rviz_config", rviz_config),
                            ("use_sim_time", use_sim_time),
                            ("log_level", log_level),
                        ],
                    ),
                ],
            )
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # ros_gz_sim_create
        RegisterEventHandler(
            OnProcessExit(
                target_action=xacro2sdf,
                on_exit=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        output="log",
                        arguments=[
                            "-file",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare(description_package),
                                    sdf_model_filepath,
                                ]
                            ),
                            "--ros-args",
                            "--log-level",
                            log_level,
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                    # ros_gz_bridge (clock -> ROS 2)
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        output="log",
                        arguments=[
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                            "--ros-args",
                            "--log-level",
                            log_level,
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                ],
            )
        ),
    ]

    return LaunchDescription(
        declared_arguments + processes + launch_descriptions + nodes
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Location of xacro/URDF to visualise
        DeclareLaunchArgument(
            "description_package",
            default_value="panda_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "sdf_model_filepath",
            default_value=path.join("panda", "model.sdf"),
            description="Path to SDF description of the robot, relative to share of `description_package`.",
        ),
        # SDF world for Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Name or filepath of the Gazebo world to load.",
        ),
        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="panda",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value=[LaunchConfiguration("name"), "_"],
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
        ),
        # Collision geometry
        DeclareLaunchArgument(
            "collision_arm",
            default_value="true",
            description="Flag to enable collision geometry for the arm.",
        ),
        DeclareLaunchArgument(
            "collision_gripper",
            default_value="true",
            description="Flag to enable collision geometry for the gripper.",
        ),
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control_command_interface",
            default_value="position",
            description="The output control command interface provided by ros2_control ('position', 'velocity', 'effort' or certain combinations 'position,velocity').",
        ),
        # Gazebo
        DeclareLaunchArgument(
            "gazebo_preserve_fixed_joint",
            default_value="false",
            description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("panda_moveit_config"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]

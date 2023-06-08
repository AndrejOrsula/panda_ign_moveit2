# panda_ign_moveit2

Software packages for Franka Emika Panda that enable manipulation with MoveIt 2 inside ~~Ignition~~ Gazebo. For control, [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) is used.

<p align="left" float="middle">
  <img width="50.0%" src="https://user-images.githubusercontent.com/22929099/147374612-3d0209d3-574e-4a4f-8077-edbbcf8fc47d.gif" alt="Animation of ex_follow_target"/>
</p>

## Overview

This branch targets ROS 2 `humble` and Gazebo `fortress`.

Below is an overview of the included packages, with a short description of their purpose. For more information, please see README.md of each individual package.

- [**panda**](./panda) – Metapackage
- [**panda_description**](./panda_description) – URDF and SDF description of the robot
- [**panda_moveit_config**](./panda_moveit_config) – MoveIt 2 configuration for the robot

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Humble](https://docs.ros.org/en/humble/Installation.html)
- Gazebo [Fortress](https://gazebosim.org/docs/fortress)

All additional dependencies are either pulled via [vcstool](https://wiki.ros.org/vcstool) ([panda_ign_moveit2.repos](./panda_ign_moveit2.repos)) or installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

### Building

Clone this repository, import dependencies, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/AndrejOrsula/panda_ign_moveit2.git
# Import dependencies
vcs import < panda_ign_moveit2/panda_ign_moveit2.repos
# Install dependencies
IGNITION_VERSION=fortress rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables:

- Execution of binaries, scripts and examples via `ros2 run panda_* <executable>`
- Launching of setup scripts via `ros2 launch panda_* <launch_script>`
- Discoverability of shared resources

# panda_ign_moveit2

This package contains configuration for Franka Emika Panda that enables its manipulation with MoveIt 2 inside Ignition Gazebo. For control, [ign_ros2_control](https://github.com/ignitionrobotics/ign_ros2_control) is used.

## Overview

Below is an overview of the included packages, with a small description of their purpose. For more information, please see README.md of each individual package.

- [**panda_description**](./panda_description) – URDF and SDF description of the robot
- [**panda_moveit_config**](./panda_moveit_config) – MoveIt 2 configuration for the robot

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work, but they were not tested.

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation.html)
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
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source panda_ws/install/local_setup.bash
```

This enables:

- Execution of scripts and examples via `ros2 run panda_* <executable>`
- Launching of setup scripts via `ros2 launch panda_* <launch_script>`
- Discoverability of shared resources

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

- ROS 2 [Rolling](https://docs.ros.org/en/rolling/Installation.html)
  - [Galactic](https://docs.ros.org/en/galactic/Installation.html) should also work without any issues (not tested)
- Ignition [Fortress](https://ignitionrobotics.org/docs/fortress)
  - [Citadel](https://ignitionrobotics.org/docs/citadel) and [Edifice](https://ignitionrobotics.org/docs/edifice) should also work (not tested)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary)
  - Install/build a version based on the selected ROS 2 release

Furthermore, the following packages are required.

- [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2)
  - Install/build a version based on the selected combination of ROS 2 release and Ignition version
- [ign_ros2_control](https://github.com/ignitionrobotics/ign_ros2_control)
  - Build a version based on the selected combination of ROS 2 release and Ignition version

Until [ros2_controllers#225](https://github.com/ros-controls/ros2_controllers/pull/225) is merged and released, `ros2_controllers` must be built from source in order to enable the use of effort command interface inside Ignition Gazebo.

- [AndrejOrsula/ros2_controllers:jtc_effort](https://github.com/AndrejOrsula/ros2_controllers/tree/jtc_effort) was tested and can be used for this purpose

### Building

Clone this repository. Then install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Create workspace for the project (can be skippid)
mkdir -p panda_ws/src && cd panda_ws
# Clone this repository
git clone https://github.com/AndrejOrsula/panda_ign_moveit2.git src/panda_ign_moveit2
# Install external dependencies via rosdep
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Build with colcon
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

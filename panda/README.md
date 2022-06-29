# panda

Metapackage for Franka Emika Panda.

## Functionality

During the build stage, this package converts xacros of [panda_description](../panda_description) and [panda_moveit_config](../panda_moveit_config) into auto-generated URDF, SDF and SRDF descriptions for convenience.

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── CMakeLists.txt # Colcon-enabled CMake recipe
└── package.xml    # ROS 2 package metadata
```

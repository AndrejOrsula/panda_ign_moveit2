# panda_moveit_config

MoveIt configuration for Franka Emika Panda.

## Instructions

### move_group

In order to configure and setup `move_group` of MoveIt 2 to plan motions inside a simulation (real robot is not yet supported), [move_group.launch.py](./launch/move_group.launch.py) script can be launched or included in another launch script.

```bash
ros2 launch panda_moveit_config move_group.launch.py <arg_i>:=<val_i>
```

To see all arguments, please use `ros2 launch --show-args panda_moveit_config move_group.launch.py`.

### SRDF

For SRDF, [panda.srdf.xacro](./srdf/panda.srdf.xacro) is the primary descriptor that includes all other xacros and creates a configuration based on the passed arguments. To generate SRDF out of xacro, you can use the included [xacro2srdf.bash](./scripts/xacro2srdf.bash) script and modify its arguments as needed. Once executed, [panda.srdf](./srdf/panda.srdf) will automatically be replaced. Alternatively, `xacro panda.srdf.xacro name:="panda" <arg_i>:=<val_i> ...` can be executed directly, e.g. this is preferred within any launch script.

## Examples

### fake_control

To see if everything works in an isolated environment, try using [ex_fake_control.launch.py](./launch/ex_fake_control.launch.py) script that allows planning motions with MoveIt 2 and executing them with fake controllers inside RViz2.

```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

### ign_control

For example inside Gazebo, try using [ex_ign_control.launch.py](./launch/ex_ign_control.launch.py) script that allows planning motions with MoveIt 2 and executing them with simulated controllers.

```bash
ros2 launch panda_moveit_config ex_ign_control.launch.py
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── config/                            # [dir] Configuration files for MoveIt 2
    ├── controllers_*.yaml             # Configuration of ROS 2 controllers for different command interfaces
    ├── joint_limits.yaml              # List of velocity and acceleration joint limits
    ├── kinematics.yaml                # Configuration for the kinematic solver
    ├── moveit_controller_manager.yaml # List of controllers with their type and action namespace for use with MoveIt 2
    ├── ompl_planning.yaml             # Configuration of OMPL planning and specific planners
    └── servo.yaml                     # Configuration for moveit_servo
├── launch/                            # [dir] ROS 2 launch scripts
    ├── ex_fake_control.launch.py      # Launch script virtual motion planning and execution inside RViz2
    └── move_group.launch.py           # Launch script for configuring and setting up move_group of MoveIt 2
├── rviz/moveit.rviz                   # RViz2 config for motion planning with MoveIt 2
├── scripts/                           # [dir] Additional useful scripts
├── srdf/                              # [dir] SRDF description (xacros)
    ├── panda_arm.xacro                # Macro for SRDF portion of Franka Emika Panda arm
    ├── panda_gripper.xacro            # Macro for SRDF portion of Franka Emika Panda gripper
    ├── panda.srdf                     # SRDF generated from `panda.srdf.xacro`
    └── panda.srdf.xacro               # The primary xacro of the robot that combines both arm and gripper
├── CMakeLists.txt                     # Colcon-enabled CMake recipe
└── package.xml                        # ROS 2 package metadata
```

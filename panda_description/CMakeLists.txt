cmake_minimum_required(VERSION 3.5)
project(panda_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Setup environment hooks that automatically prepend IGN_GAZEBO_RESOURCE_PATH and SDF_PATH when sourcing local_setup.<ext>
ament_environment_hooks(
  "cmake/environment_hooks/ign_gazebo_resource_path.bash"
  "cmake/environment_hooks/sdf_path.bash"
)

set(SCRIPTS_DIR scripts)
install(PROGRAMS
    ${SCRIPTS_DIR}/xacro2sdf_direct.bash
    ${SCRIPTS_DIR}/xacro2sdf.bash
    ${SCRIPTS_DIR}/xacro2urdf.bash
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY config launch panda panda/meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()

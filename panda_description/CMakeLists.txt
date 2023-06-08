cmake_minimum_required(VERSION 3.5)
project(panda_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

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

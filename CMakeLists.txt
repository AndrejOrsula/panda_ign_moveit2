cmake_minimum_required(VERSION 3.5)
project(panda_ign)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY urdf panda/meshes DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()

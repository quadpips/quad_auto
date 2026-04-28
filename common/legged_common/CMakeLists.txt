cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(legged_common)

# ament_cmake dependencies
set(dependencies
    rclcpp
    legged_utils
)

# Find ROS2 build system
find_package(ament_cmake REQUIRED)

# External dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(proxsuite REQUIRED)

# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(legged_utils REQUIRED)
find_package(pinocchio REQUIRED)

# Need now, not sure why
find_package(Boost REQUIRED COMPONENTS 
              system 
              filesystem
              log 
              log_setup)

###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Export
ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(go2_estimators)

# ament_cmake dependencies
set(dependencies
    rclcpp
    legged_common
    legged_utils
    ocs2_custom_quadruped_interface
    ocs2_go2_mpc
)

# Find ROS2 build system
find_package(ament_cmake REQUIRED)

# External dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(proxsuite REQUIRED)

# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(legged_common REQUIRED)
find_package(legged_utils REQUIRED)
find_package(ocs2_custom_quadruped_interface REQUIRED)
find_package(ocs2_go2_mpc REQUIRED)

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

# Source files
set(SOURCES
  src/estimators/NoiseParams.cpp
  src/estimators/StateEstimatorContainer.cpp
  src/estimators/CheaterPositionVelocityEstimator.cpp
  src/estimators/InEKF.cpp
  src/estimators/InEKFEstimator.cpp
  src/estimators/LieGroup.cpp
  src/estimators/RobotState.cpp
  src/observers/OrientationObserver.cpp
)

# Build the library
add_library(${PROJECT_NAME} ${SOURCES})

# Declare dependencies
ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
	message(STATUS "Run clang tooling")
	add_clang_tooling(
		TARGETS ${PROJECT_NAME}
		SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
		CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
		CF_WERROR
	)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Install built library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Export
ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
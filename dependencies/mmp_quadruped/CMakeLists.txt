cmake_minimum_required(VERSION 3.15)
project(mmp_quadruped)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS2 build system
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(ocs2_switched_model_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ocs2_custom_quadruped_interface REQUIRED)
find_package(ocs2_go2_mpc REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)
find_package(PCL REQUIRED)

# Need now, not sure why
find_package(Boost REQUIRED COMPONENTS 
              system 
              filesystem
              log 
              log_setup)

set(dependencies
        rclcpp
        ocs2_core
        ocs2_robotic_tools
        convex_plane_decomposition
        ocs2_switched_model_interface
        tf2_ros
        tf2_geometry_msgs
        ocs2_custom_quadruped_interface
        ocs2_go2_mpc
)

###########
## Build ##
###########

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
        )

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
  src/environment/EnvironmentProcessor.cpp
  src/planning/ConstraintManager.cpp
  src/planning/ContactSequence.cpp
  src/planning/Graph.cpp
  src/planning/GraphSearcher.cpp
  src/planning/GraphTraverser.cpp
  src/planning/ModeFamily.cpp
  src/planning/ModeFamilyNode.cpp
  src/planning/MultiModalPlanner.cpp
  src/planning/ReferenceTrajectoryPublisher.cpp
  src/planning/Superquadric.cpp
  src/planning/TorsoPathPlanner.cpp
  src/planning/TorsoStateNode.cpp
  src/visualization/Go2Visualizer.cpp
  src/visualization/MultiModalVisualizer.cpp
  src/PolygonPublisher.cpp
  src/TerrainPublisher.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
	${dependencies}
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        jsoncpp_lib
)
        
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})
     
## Declare a C++ executable

# Multi-modal single-mode plan node
add_executable(go2_mmp
        src/LeggedRobotMmpNode.cpp
)
ament_target_dependencies(go2_mmp
	${dependencies}
)
target_link_libraries(go2_mmp
        ${PROJECT_NAME}
)
target_compile_options(go2_mmp PRIVATE ${OCS2_CXX_FLAGS})


# Publish the terrain for the environment
add_executable(terrain_publisher
        src/PublishTerrain.cpp
)
ament_target_dependencies(terrain_publisher
	${dependencies}
)
target_link_libraries(terrain_publisher
        ${PROJECT_NAME}
)

# Publish the polygons for the environment
add_executable(polygon_publisher
        src/PublishPolygon.cpp
        )
ament_target_dependencies(polygon_publisher
	${dependencies}
)
target_link_libraries(polygon_publisher
        ${PROJECT_NAME}
        )

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS 
        go2_mmp
        terrain_publisher
        polygon_publisher
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
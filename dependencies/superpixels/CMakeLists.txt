cmake_minimum_required(VERSION 3.15)
project(superpixels)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
    rclcpp
    cv_bridge
    image_transport
    pcl_ros
    convex_plane_decomposition
    convex_plane_decomposition_ros
    convex_plane_decomposition_msgs
    segmented_planes_terrain_model
    egocylindrical
    )


# Dependencies
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL    REQUIRED)

find_package(OpenMP)

# Need now, not sure why
find_package(Boost REQUIRED COMPONENTS 
              system 
              filesystem
              log 
              log_setup)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
        src/ConvexHullifier.cpp
        src/ImagePreprocessor.cpp
        src/Ransac.cpp
        src/SuperpixelDepthSegmenter.cpp
        src/Visualizer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

## Declare a C++ executable

# Superpixel algorithm node for depth images (custom implementation)
add_executable(superpixel_depth_segmentation_node
        src/SuperpixelDepthSegmentationNode.cpp
        )
ament_target_dependencies(superpixel_depth_segmentation_node
	${dependencies}
)
target_link_libraries(superpixel_depth_segmentation_node
        ${PROJECT_NAME}
        ${OpenMP_CXX_FLAGS}
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
        superpixel_depth_segmentation_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY cfg launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
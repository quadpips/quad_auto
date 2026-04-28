cmake_minimum_required(VERSION 3.15)
project(egocylindrical)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# ament_cmake dependencies
set(dependencies
      cv_bridge
      geometry_msgs
      visualization_msgs
      image_geometry
      image_transport
      egocylindrical_msgs
      # nodelet
      pcl_ros
      rclcpp
      rclpy
      sensor_msgs
      std_msgs
      # tf
      tf2
      tf2_msgs
      tf2_ros
      tf2_sensor_msgs
      tf2_geometry_msgs
      rosidl_default_generators
      depth_img_normal_estimation
      # message_generation
      # dynamic_reconfigure
      # benchmarking_tools #Probably just temporary to avoid adding any custom dependencies to this package.
)

# Dependencies
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)
find_package(OpenMP)

if(0)
    set(SHOW_VECTORIZE_INFO "-fopt-info-vec-all=vectorized.txt")
else()
    set(SHOW_VECTORIZE_INFO "")
endif()


if(0) #if on arm...
    set(PLATFORM_FLAGS "PIPS_ON_ARM=arm")
else()
    set(PLATFORM_FLAGS "")
endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder

# generate_dynamic_reconfigure_options(
#   cfg/Propagator.cfg
#   cfg/RangeImageGenerator.cfg
#   cfg/Filter.cfg
#   cfg/RangeImageInflatorGenerator.cfg
#   cfg/FloorImageGenerator.cfg
#   )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES egocylindrical_propagator range_image_core can_image_core floor_image_core point_cloud_core projected_point_cloud_core semantic_point_cloud_core normal_point_cloud_core normal_point_cloud_floor_only_core point_transformer depth_image_remapper range_image_converter depth_image_inserter laser_scan_inserter sensors ecwrapper_buffer
#   CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport roscpp rospy sensor_msgs std_msgs tf tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs 
#  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###############
## Libraries ##
###############

# RANGE_IMAGE_CORE
add_library(range_image_core 
            src/range_image_core.cpp)
ament_target_dependencies(range_image_core 	
                           ${dependencies}
                           ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_compile_options(range_image_core PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_range_image_core.txt -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(range_image_core
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

# CAN_IMAGE_CORE
add_library(can_image_core 
            src/can_image_core.cpp)
ament_target_dependencies(can_image_core 
                  ${dependencies}
                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_compile_options(can_image_core PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_can_image_core.txt -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(can_image_core
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

# FLOOR_IMAGE_CORE
add_library(floor_image_core src/floor_image_core.cpp)
ament_target_dependencies(floor_image_core 
                          ${dependencies}
                          ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_options(floor_image_core PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_floor_image_core.txt -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(floor_image_core
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
  )

# POINT_CLOUD_CORE
add_library(point_cloud_core src/point_cloud_core.cpp)
ament_target_dependencies(point_cloud_core
                  ${dependencies}
                  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_options(point_cloud_core PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_point_cloud_core.txt ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(point_cloud_core)

# PROJECTED_POINT_CLOUD_CORE
add_library(projected_point_cloud_core src/projected_point_cloud_core.cpp)
ament_target_dependencies(projected_point_cloud_core ${dependencies} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_options(projected_point_cloud_core PRIVATE -ftree-vectorize -fno-trapping-math -fno-math-errno  -fopt-info-loop-optimized=vec_projected_point_cloud_core.txt ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(projected_point_cloud_core)


add_library(semantic_point_cloud_core src/semantic_point_cloud_core.cpp)
ament_target_dependencies(semantic_point_cloud_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(semantic_point_cloud_core PRIVATE -ftree-vectorize -fno-trapping-math -fno-math-errno  -fopt-info-loop-optimized=vec_projected_point_cloud_core.txt ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(semantic_point_cloud_core)

add_library(normal_point_cloud_core src/normal_point_cloud_core.cpp)
ament_target_dependencies(normal_point_cloud_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(normal_point_cloud_core PRIVATE -ftree-vectorize -fno-trapping-math -fno-math-errno  -fopt-info-loop-optimized=vec_projected_point_cloud_core.txt ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(normal_point_cloud_core)

add_library(normal_point_cloud_floor_only_core src/normal_point_cloud_floor_only_core.cpp)
ament_target_dependencies(normal_point_cloud_floor_only_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(normal_point_cloud_floor_only_core PRIVATE -ftree-vectorize -fno-trapping-math -fno-math-errno  -fopt-info-loop-optimized=vec_projected_point_cloud_core.txt ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(normal_point_cloud_floor_only_core)

add_library(point_transformer src/point_transformer.cpp)
ament_target_dependencies(point_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(point_transformer PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_point_transformer.txt -funsafe-math-optimizations -fno-math-errno ${OpenMP_CXX_FLAGS} #-finite-math-only and -fno-trapping-math. -ffinite-math-only 
   )
target_compile_definitions(point_transformer PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(point_transformer
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

add_library(point_propagator src/point_propagator.cpp)
target_compile_definitions(point_propagator PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(point_propagator
   point_transformer
   ${catkin_LIBRARIES}
   )

add_library(depth_image_remapper src/depth_image_remapper.cpp)
ament_target_dependencies(depth_image_remapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(depth_image_remapper PRIVATE -ftree-vectorize  -fopt-info-vec-all=vec_depth_image_remapper.txt -funsafe-math-optimizations -fno-math-errno ${OpenMP_CXX_FLAGS}#-finite-math-only and -fno-trapping-math. -ffinite-math-only 
   )
target_compile_definitions(depth_image_remapper PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(depth_image_remapper
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

add_library(range_image_dilator src/range_image_dilator_generator.cpp
            src/range_image_dilator_core.cpp)
ament_target_dependencies(range_image_dilator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(range_image_dilator PRIVATE)
target_link_libraries(range_image_dilator)

#RANGE_IMAGE_CONVERTER
add_library(range_image_converter src/range_image_converter.cpp)
ament_target_dependencies(range_image_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_link_libraries(range_image_converter
   ${OpenMP_CXX_FLAGS}
   )
   
# DEPTH_IMAGE_DIFFERENCE
add_library(depth_image_difference src/depth_image_difference.cpp
            src/depth_image_difference_debugging_publisher.cpp)
ament_target_dependencies(depth_image_difference ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(depth_image_difference PRIVATE)
target_compile_definitions(depth_image_difference PRIVATE ${PLATFORM_FLAGS} )
target_link_libraries(depth_image_difference
   range_image_core
   range_image_dilator
   range_image_converter
   point_cloud_core
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

# DEPTH_IMAGE_INSERTER
add_library(depth_image_inserter src/depth_image_inserter.cpp)
ament_target_dependencies(depth_image_inserter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(depth_image_inserter PRIVATE -ftree-vectorize  -fopt-info-vec-all=vec_depth_image_inserter.txt -funsafe-math-optimizations -fno-math-errno ${OpenMP_CXX_FLAGS}#-finite-math-only and -fno-trapping-math. -ffinite-math-only 
  )
target_compile_definitions(depth_image_inserter PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(depth_image_inserter
   depth_image_remapper
   depth_image_difference
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

# LASER_SCAN_INSERTER
add_library(laser_scan_inserter src/laser_scan_inserter.cpp)
ament_target_dependencies(laser_scan_inserter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
#target_compile_options(laser_scan_inserter PRIVATE -O0 -g)
target_link_libraries(laser_scan_inserter
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

# SENSORS
add_library(sensors src/sensor.cpp src/sensor_collection.cpp)
#target_compile_options(sensors PRIVATE -O0 -g)
target_link_libraries(sensors
   ##TODO: add the various sensor libraries here once implementations moved to source files
   depth_image_inserter
   laser_scan_inserter
   )

add_library(ecwrapper_buffer src/ecwrapper_buffer.cpp)
ament_target_dependencies(ecwrapper_buffer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
#target_compile_options(laser_scan_inserter PRIVATE -O0 -g)
target_link_libraries(ecwrapper_buffer
   )

#EGOCYLINDRICAL_PROPAGATOR
add_library(egocylindrical_propagator src/egocylindrical_propagator.cpp)
ament_target_dependencies(egocylindrical_propagator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})
target_compile_options(egocylindrical_propagator PRIVATE -g -ftree-vectorize -funsafe-math-optimizations ${OpenMP_CXX_FLAGS}
   )
target_compile_definitions(egocylindrical_propagator PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(egocylindrical_propagator
   point_propagator
   sensors
   ecwrapper_buffer
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )
   
#EGOCYLINDRICAL_PROPAGATOR_NODE
add_executable(egocylindrical_propagator_node src/egocylindrical_propagator_node.cpp)
target_link_libraries(egocylindrical_propagator_node
   egocylindrical_propagator
   )

#POINT_CLOUD_GENERATOR
add_library(point_cloud_generator src/point_cloud_generator.cpp)
target_link_libraries(point_cloud_generator
   point_cloud_core
   )

#POINT_CLOUD_NODE
add_executable(point_cloud_node src/point_cloud_node.cpp)
target_link_libraries(point_cloud_node
   point_cloud_generator
   )

#PROJECTED_POINT_CLOUD_GENERATOR
add_library(projected_point_cloud_generator src/projected_point_cloud_generator.cpp)
target_link_libraries(projected_point_cloud_generator
   projected_point_cloud_core
   )

#PROJECTED_POINT_CLOUD_NODE
add_executable(projected_point_cloud_node src/projected_point_cloud_node.cpp)
target_link_libraries(projected_point_cloud_node
   projected_point_cloud_generator
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

#SEMANTIC_POINT_CLOUD_GENERATOR
add_library(semantic_point_cloud_generator src/semantic_point_cloud_generator.cpp)
target_link_libraries(semantic_point_cloud_generator
   semantic_point_cloud_core
   )

#SEMANTIC_POINT_CLOUD_NODE
add_executable(semantic_point_cloud_node src/semantic_point_cloud_node.cpp)
target_link_libraries(semantic_point_cloud_node
   semantic_point_cloud_generator
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

#NORMAL_POINT_CLOUD_GENERATOR
add_library(normal_point_cloud_generator src/normal_point_cloud_generator.cpp)
target_link_libraries(normal_point_cloud_generator
   normal_point_cloud_core
   )

#NORMAL_POINT_CLOUD_NODE
add_executable(normal_point_cloud_node src/normal_point_cloud_node.cpp)
target_link_libraries(normal_point_cloud_node
   normal_point_cloud_generator
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
)

#FLOOR_IMAGE_GENERATOR
add_library(floor_image_generator src/floor_image_generator.cpp)
target_link_libraries(floor_image_generator
   range_image_core
   floor_image_core
   ${OpenMP_CXX_FLAGS}
)

#FLOOR_IMAGE_NODE
add_executable(floor_image_node src/floor_image_node.cpp)
target_link_libraries(floor_image_node
   floor_image_generator
)

#RANGE_IMAGE_GENERATOR
add_library(range_image_generator src/range_image_generator.cpp)
target_link_libraries(range_image_generator
   range_image_core
   can_image_core
   ${OpenMP_CXX_FLAGS}
)

#RANGE_IMAGE_NODE
add_executable(range_image_node src/range_image_node.cpp)
target_link_libraries(range_image_node
   range_image_generator
)
  

#############
## Install ##
#############


install(
  TARGETS range_image_core can_image_core floor_image_core point_cloud_core 
            projected_point_cloud_core semantic_point_cloud_core normal_point_cloud_core
            normal_point_cloud_floor_only_core point_transformer point_propagator
            depth_image_remapper range_image_dilator range_image_converter
            laser_scan_inserter depth_image_inserter depth_image_difference
            sensors ecwrapper_buffer egocylindrical_propagator point_cloud_generator
            projected_point_cloud_generator semantic_point_cloud_generator
            normal_point_cloud_generator floor_image_generator range_image_generator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS 
      egocylindrical_propagator_node
      point_cloud_node
      projected_point_cloud_node
      semantic_point_cloud_node
      normal_point_cloud_node
      floor_image_node
      range_image_node
      DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY cfg launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
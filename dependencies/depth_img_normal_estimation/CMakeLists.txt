cmake_minimum_required(VERSION 3.15)
project(depth_img_normal_estimation)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(dependencies
        rclcpp
        cv_bridge
        image_transport
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
    src/NormalEstimator.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
                                ${dependencies})

target_link_libraries(${PROJECT_NAME}
                       yaml-cpp
                       )

# target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

## Declare a C++ executable

# Normal estimation node
add_executable(${PROJECT_NAME}_node src/NormalEstimationNode.cpp )

ament_target_dependencies(${PROJECT_NAME}_node
                                ${dependencies}
)
target_compile_options(${PROJECT_NAME}_node PRIVATE 
                        -ftree-vectorize 
                        -fopt-info-loop-optimized=vec_${PROJECT_NAME}_node.txt 
                        -funsafe-math-optimizations 
                        -fno-math-errno 
                        -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(${PROJECT_NAME}_node
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
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS 
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)       
install(DIRECTORY cfg launch
        DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############

# ament_export_dependencies(${dependencies})  
# ament_export_include_directories("include/${PROJECT_NAME}")
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
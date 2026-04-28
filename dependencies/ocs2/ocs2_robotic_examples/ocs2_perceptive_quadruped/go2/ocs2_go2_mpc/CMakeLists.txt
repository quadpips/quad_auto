cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_go2_mpc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_go2_models REQUIRED)
find_package(ocs2_custom_quadruped_interface REQUIRED) 

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PCL REQUIRED)


find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

set(dependencies
  ocs2_go2_models
  ocs2_custom_quadruped_interface
  Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/Go2Interface.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# MPC executable
add_executable(${PROJECT_NAME}_mpc_node
  src/Go2MpcNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}_mpc_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
)

# MPC debugging executable
add_executable(${PROJECT_NAME}_mpc_mrt_debugging_node
  src/MpcMrtDebugging.cpp
)
ament_target_dependencies(${PROJECT_NAME}_mpc_mrt_debugging_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_mpc_mrt_debugging_node
  ${PROJECT_NAME}
)

# dummy executable
add_executable(${PROJECT_NAME}_dummy_mrt_node
  src/Go2DummyMRT.cpp
)
ament_target_dependencies(${PROJECT_NAME}_dummy_mrt_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
)

# dummy executable
add_executable(${PROJECT_NAME}_mpc_mrt_node
  src/Go2MpcMrtNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}_mpc_mrt_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_mpc_mrt_node
  ${PROJECT_NAME}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_mpc_node ${PROJECT_NAME}_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

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
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS 
  ${PROJECT_NAME}_mpc_node 
  ${PROJECT_NAME}_mpc_mrt_debugging_node
  ${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}_mpc_mrt_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
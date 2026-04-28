cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(go2_interface)

find_package(ament_cmake REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(go2_interface_msgs REQUIRED)
find_package(go2_system REQUIRED)
find_package(go2_estimators REQUIRED)

# External dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PCL REQUIRED)

# Need now, not sure why
find_package(Boost REQUIRED COMPONENTS 
              system 
              filesystem
              log_setup
              log 
)

set(dependencies
    legged_common
    go2_interface_msgs
    go2_estimators
    go2_system
    Boost
)

###########
## Build ##
###########

include_directories(include 
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

# Main library
add_library(${PROJECT_NAME}   
            src/Go2GazeboBridge.cpp
            src/Go2System.cpp
)
# Declare dependencies for build/export
ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Gazebo sim executable
add_executable(go2_gazebo_main src/mainGo2Gazebo.cpp)
ament_target_dependencies(go2_gazebo_main
	${dependencies}
)
target_link_libraries(go2_gazebo_main ${PROJECT_NAME} )

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

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS 
        go2_gazebo_main
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
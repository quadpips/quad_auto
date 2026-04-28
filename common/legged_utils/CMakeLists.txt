cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(legged_utils)

# Dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
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

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

############
## Export ##
############
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()
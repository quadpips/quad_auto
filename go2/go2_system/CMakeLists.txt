cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(go2_system)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(legged_utils REQUIRED)
find_package(legged_common REQUIRED)
find_package(ocs2_go2_mpc REQUIRED)
find_package(unitree_go REQUIRED)
find_package(go2_estimators REQUIRED)
find_package(go2_wbc REQUIRED)

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
  legged_utils
  go2_wbc
  go2_estimators
  unitree_go
  Boost
)

###########
## Build ##
###########

include_directories(include     
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
        src/controllers/auxillary/LegController.cpp
        src/state_machine/ControlFSM.cpp
        src/state_machine/FSM_State_Passive.cpp
        src/state_machine/FSM_State_PerceptiveLocomotionOCS2.cpp
        src/state_machine/FSM_State_RecoveryStand.cpp
        src/state_machine/FSM_State.cpp
        src/state_machine/SafetyChecker.cpp
)

# Declare dependencies for build/export
ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

############
## Export ##
############

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
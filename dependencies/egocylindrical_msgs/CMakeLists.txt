cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(egocylindrical_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

##############################
## Declare custom messageas ##
##############################

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/EgoCylinderPoints.msg"
   DEPENDENCIES 
        builtin_interfaces
        std_msgs 
        rosidl_default_generators
)

#############
## Install ##
#############

# Install message and service files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
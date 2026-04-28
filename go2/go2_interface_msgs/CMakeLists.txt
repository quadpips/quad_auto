cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(go2_interface_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the message files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/GO2Cmd.srv"
        "srv/CheaterMode.srv"        
        DEPENDENCIES 
        builtin_interfaces
        std_msgs 
        geometry_msgs 
        sensor_msgs
        rosidl_default_generators
)

# Install message and service files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
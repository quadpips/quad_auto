cmake_minimum_required(VERSION 3.5)
project(go2_gazebo)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  urdf
  gazebo_ros2_control
  gazebo_ros
  sensor_msgs
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(go2_foot_contact_sensor src/FootContactSensor.cpp)
target_link_libraries(go2_foot_contact_sensor ${GAZEBO_LIBRARIES})
ament_target_dependencies(go2_foot_contact_sensor ${dependencies})

#############
## Install ##
#############

install(TARGETS 
  go2_foot_contact_sensor
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()

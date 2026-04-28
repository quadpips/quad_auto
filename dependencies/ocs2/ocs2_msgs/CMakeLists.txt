cmake_minimum_required(VERSION 3.5)
project(ocs2_msgs)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MpcState.msg"
  "msg/MpcInput.msg"
  "msg/ModeSchedule.msg"
  "msg/MpcObservation.msg"
  "msg/MpcPerformanceIndices.msg"
  "msg/MpcTargetTrajectories.msg"
  "msg/ControllerData.msg"
  "msg/MpcFlattenedController.msg"
  "msg/LagrangianMetrics.msg"
  "msg/Multiplier.msg"
  "msg/Constraint.msg"
  "srv/Reset.srv"
  DEPENDENCIES std_msgs
)

# Install message and service files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}
)

# Install generated headers
install(DIRECTORY ${ament_index_include_dirs} 
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

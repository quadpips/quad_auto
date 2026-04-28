cmake_minimum_required(VERSION 3.16)
project(unitree_joint_controller)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_toolbox
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    urdf
    unitree_go
)

find_package(ament_cmake REQUIRED)
# find_package(backward_ros REQUIRED) # not sure if needed
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(unitree_joint_controller_parameters
  src/unitree_joint_controller_parameters.yaml
  include/unitree_joint_controller/validate_ujc_parameters.hpp
)

add_library(unitree_joint_controller SHARED
  src/unitree_joint_controller.cpp
)
target_compile_features(unitree_joint_controller PUBLIC cxx_std_17)
target_include_directories(unitree_joint_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/unitree_joint_controller>
)
target_link_libraries(unitree_joint_controller PUBLIC
  unitree_joint_controller_parameters
)
ament_target_dependencies(unitree_joint_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface unitree_joint_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_unitree_joint_controller
    test/test_load_unitree_joint_controller.cpp
  )
  target_link_libraries(test_load_unitree_joint_controller
    unitree_joint_controller
  )
  ament_target_dependencies(test_load_unitree_joint_controller
    controller_manager
    control_toolbox
    realtime_tools
    ros2_control_test_assets
  )
endif()


install(
  DIRECTORY include/
  DESTINATION include/unitree_joint_controller
)
install(TARGETS
    unitree_joint_controller
    unitree_joint_controller_parameters
  EXPORT export_unitree_joint_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_unitree_joint_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

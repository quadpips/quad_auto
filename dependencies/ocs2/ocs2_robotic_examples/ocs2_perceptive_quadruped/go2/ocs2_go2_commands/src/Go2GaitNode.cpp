/*
 * Go2ModeSequenceCommand.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ocs2_go2_commands/ModeSequenceKeyboard.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) 
{
  const std::string robotName = "go2";
  // std::string gaitFile =
  //     ament_index_cpp::get_package_share_directory("ocs2_go2_commands") +
  //     "/config/gait.info";
  // std::cerr << "Loading gait file: " << gaitFile << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robotName + "_gait_command_node",
                                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                                .allow_undeclared_parameters(true));

  const std::string gaitFile = node->get_parameter("gaitCommandFile").as_string();

  switched_model::ModeSequenceKeyboard modeSequenceCommand(node, gaitFile,
                                                           robotName, true);

  while (rclcpp::ok()) 
  {
    modeSequenceCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}

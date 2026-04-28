/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) 
{
  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("anymal_mrt");

  // std::string taskFile, frameFile, urdfFile, sqpFile; // , envFile

  node->declare_parameter("taskFile", "N/A");
  const std::string taskFile =
      node->get_parameter("taskFile").as_string();

  node->declare_parameter("urdfFile", "N/A");
  const std::string urdfFile =
      node->get_parameter("urdfFile").as_string();

  node->declare_parameter("frameFile", "N/A");
  const std::string frameFile =
      node->get_parameter("frameFile").as_string();

  node->declare_parameter("sqpFile", "N/A");
  const std::string sqpFile =
      node->get_parameter("sqpFile").as_string();      

  // nodeHandle.getParam("/taskFile", taskFile);
  // nodeHandle.getParam("/urdfFile", urdfFile);
  // nodeHandle.getParam("/frameFile", frameFile);
  // nodeHandle.getParam("/envFile", envFile);
  // nodeHandle.getParam("/sqpFile", sqpFile);
  
  std::string urdfString = anymal::getUrdfString(urdfFile);

  auto anymalInterface = anymal::getAnymalInterface(urdfString, taskFile, frameFile); // , envFile
  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile); // new

  quadrupedDummyNode(node, 
                     *anymalInterface, 
                     &anymalInterface->getRollout(), 
                     mpcSettings.mrtDesiredFrequency_,
                     mpcSettings.mpcDesiredFrequency_);

  return 0;
}

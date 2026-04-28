/*
 * AnymalMPCNode.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedMpc.h>
#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) 
{
  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("anymal_mpc");


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
  
  std::string urdfString = anymal::getUrdfString(urdfFile);

  auto anymalInterface = anymal::getAnymalInterface(urdfString, taskFile, frameFile); // , envFile
  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile); // new

  switch (anymalInterface->modelSettings().algorithm_) 
  {
    case switched_model::Algorithm::DDP: 
    {
      const auto ddpSettings = ocs2::ddp::loadSettings(taskFile); // new
      auto mpcPtr = getDdpMpc(*anymalInterface, mpcSettings, ddpSettings);
      quadrupedMpcNode(node, *anymalInterface, std::move(mpcPtr));
      break;
    }
    case switched_model::Algorithm::SQP: 
    {
      ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile); // new
      auto mpcPtr = getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
      quadrupedMpcNode(node, *anymalInterface, std::move(mpcPtr));
      break;
    }
  }

  return 0;
}

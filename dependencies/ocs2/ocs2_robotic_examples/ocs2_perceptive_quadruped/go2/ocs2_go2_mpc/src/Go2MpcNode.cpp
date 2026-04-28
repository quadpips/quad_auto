/*
 * Go2MPCNode.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedMpc.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedMpcNode.h>

#include "ocs2_go2_mpc/Go2Interface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) 
{
  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("go2_mpc");


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
  
  std::string urdfString = go2::getUrdfString(urdfFile);

  auto go2Interface = go2::getGo2Interface(urdfString, taskFile, frameFile); // , envFile
  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile); // new

  switch (go2Interface->modelSettings().algorithm_) 
  {
    case switched_model::Algorithm::DDP: 
    {
      const auto ddpSettings = ocs2::ddp::loadSettings(taskFile); // new
      auto mpcPtr = getDdpMpc(*go2Interface, mpcSettings, ddpSettings);
      customQuadrupedMpcNode(node, *go2Interface, std::move(mpcPtr));
      break;
    }
    case switched_model::Algorithm::SQP: 
    {
      ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile); // new
      auto mpcPtr = getSqpMpc(*go2Interface, mpcSettings, sqpSettings);
      customQuadrupedMpcNode(node, *go2Interface, std::move(mpcPtr));
      break;
    }
  }

  return 0;
}

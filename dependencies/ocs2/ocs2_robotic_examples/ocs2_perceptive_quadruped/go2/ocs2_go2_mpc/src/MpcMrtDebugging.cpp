//
// Created by rgrandia on 31.03.22.
//

#include "rclcpp/rclcpp.hpp"

#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_go2_mpc/Go2Interface.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedMpc.h>

#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_custom_quadruped_interface/CustomSwingPlanningVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainPlaneVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainReceiver.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedVisualizer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"

using namespace switched_model;
using MN = switched_model::ModeNumber;

int main(int argc, char* argv[]) 
{
  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mpc_debugging_demo");

  RCLCPP_INFO_STREAM(node->get_logger(), "Starting the MPC debugging demo");
  
  const std::string robotName = "go2";

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

  auto legged_interface_ = go2::getGo2Interface(urdfString, taskFile, frameFile);

  // ====== Create MPC solver ========

  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile); // new

  std::unique_ptr<ocs2::MPC_BASE> mpcPtr;
  switch (legged_interface_->modelSettings().algorithm_) 
  {
    case switched_model::Algorithm::DDP: 
    {
      const auto ddpSettings = ocs2::ddp::loadSettings(taskFile); // new

      mpcPtr = getDdpMpc(*legged_interface_, mpcSettings, ddpSettings);
      break;
    }
    case switched_model::Algorithm::SQP: 
    {
      ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile); // new

      mpcPtr = getSqpMpc(*legged_interface_, mpcSettings, sqpSettings);
      break;
    }
  }

  // ======= Placeholder for synchronized modules, not using yet ======== //

  // MPC

  RCLCPP_INFO_STREAM(node->get_logger(), "Creating gait");

  RCLCPP_INFO_STREAM(node->get_logger(), "Creating MPC_MRT_Interface");

  ocs2::MPC_MRT_Interface mpcInterface(*mpcPtr);

  // ====== Set the scenario to the correct interfaces ========
  auto referenceManager = legged_interface_->getSwitchedModelModeScheduleManagerPtr();

  // Create observation
  ocs2::SystemObservation observation;
  observation.time = 0.0;
  observation.mode = MN::STANCE;
  observation.state = legged_interface_->getInitialState();
  observation.input.setZero(24);

  ocs2::TargetTrajectories targetTrajectories({ observation.time }, { observation.state },
                                                { observation.input });

  // Register the target trajectory
  referenceManager->setTargetTrajectories(targetTrajectories);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);
  while (!mpcInterface.initialPolicyReceived()) 
  {
    mpcInterface.advanceMpc();
  }

  // ====== Execute the scenario ========
  ocs2::scalar_t finalTime = 5.0; // sec
  while (observation.time < finalTime) 
  {
    try 
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "observation (pre-mpc): \n");
      RCLCPP_INFO_STREAM(node->get_logger(), "   time:                " << observation.time);
      RCLCPP_INFO_STREAM(node->get_logger(), "   mode:                " << observation.mode);
      RCLCPP_INFO_STREAM(node->get_logger(), "   state: ");
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso orientation: " << observation.state.segment(0,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso position:    " << observation.state.segment(3,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso angular vel: " << observation.state.segment(6,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso linear vel:  " << observation.state.segment(9,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LF joint pos:      " << observation.state.segment(12,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RF joint pos:      " << observation.state.segment(15,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LH joint pos:      " << observation.state.segment(18,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RH joint pos:      " << observation.state.segment(21,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "   input: ");
      RCLCPP_INFO_STREAM(node->get_logger(), "     LF contact force:  " << observation.input.segment(0,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RF contact force:  " << observation.input.segment(3,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LH contact force:  " << observation.input.segment(6,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RH contact force:  " << observation.input.segment(9,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LF joint vel:      " << observation.input.segment(12,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RF joint vel:      " << observation.input.segment(15,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LH joint vel:      " << observation.input.segment(18,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RH joint vel:      " << observation.input.segment(21,3).transpose());

      // run MPC at current observation
      mpcInterface.setCurrentObservation(observation);
      mpcInterface.advanceMpc();
      mpcInterface.updatePolicy();

      // Evaluate the optimized solution - change to optimal controller
      ocs2::vector_t tmp;
      mpcInterface.evaluatePolicy(observation.time, observation.state, tmp, observation.input, observation.mode);

      RCLCPP_INFO_STREAM(node->get_logger(), "observation (post-mpc): \n");
      RCLCPP_INFO_STREAM(node->get_logger(), "   time:                " << observation.time);
      RCLCPP_INFO_STREAM(node->get_logger(), "   mode:                " << observation.mode);
      RCLCPP_INFO_STREAM(node->get_logger(), "   state: ");
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso orientation: " << observation.state.segment(0,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso position:    " << observation.state.segment(3,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso angular vel: " << observation.state.segment(6,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     torso linear vel:  " << observation.state.segment(9,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LF joint pos:      " << observation.state.segment(12,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RF joint pos:      " << observation.state.segment(15,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LH joint pos:      " << observation.state.segment(18,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RH joint pos:      " << observation.state.segment(21,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "   input: ");
      RCLCPP_INFO_STREAM(node->get_logger(), "     LF contact force:  " << observation.input.segment(0,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RF contact force:  " << observation.input.segment(3,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LH contact force:  " << observation.input.segment(6,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RH contact force:  " << observation.input.segment(9,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LF joint vel:      " << observation.input.segment(12,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RF joint vel:      " << observation.input.segment(15,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     LH joint vel:      " << observation.input.segment(18,3).transpose());
      RCLCPP_INFO_STREAM(node->get_logger(), "     RH joint vel:      " << observation.input.segment(21,3).transpose());

    } catch (std::exception& e) 
    {
      std::cout << "MPC failed\n";
      std::cout << e.what() << "\n";
      break;
    }
  }

  return 0;
}
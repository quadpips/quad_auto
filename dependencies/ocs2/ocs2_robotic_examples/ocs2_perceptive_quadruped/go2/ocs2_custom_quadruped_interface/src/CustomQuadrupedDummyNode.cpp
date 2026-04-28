//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_custom_quadruped_interface/CustomQuadrupedDummyNode.h"

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedLogger.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedVisualizer.h>

namespace switched_model {

void customQuadrupedDummyNode(const rclcpp::Node::SharedPtr &node, const CustomQuadrupedInterface& quadrupedInterface, const ocs2::RolloutBase* rolloutPtr,
                            double mrtDesiredFrequency, double mpcDesiredFrequency)
{
  const std::string robotName = "go2";

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  if (rolloutPtr != nullptr) 
  {
    mrt.initRollout(rolloutPtr);
  }
  mrt.launchNodes(node);

  // Visualization
  auto visualizer = std::make_shared<switched_model::CustomQuadrupedVisualizer>(
      quadrupedInterface.getKinematicModel(), quadrupedInterface.getJointNames(), quadrupedInterface.getBaseName(), node);

  // Logging
  std::string logFileName = "/tmp/ocs2/CustomQuadrupedDummyNodeLog.txt";
  auto logger = std::make_shared<switched_model::CustomQuadrupedLogger>(logFileName, quadrupedInterface.getKinematicModel(),
                                                                        quadrupedInterface.getComModel());

  // Dummy MRT
  ocs2::MRT_ROS_Dummy_Loop dummySimulator(mrt, mrtDesiredFrequency, mpcDesiredFrequency);
  dummySimulator.subscribeObservers({visualizer, logger});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = quadrupedInterface.getInitialState();
  initObservation.input = vector_t::Zero(INPUT_DIM);
  initObservation.mode = switched_model::ModeNumber::STANCE;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input});

  // run dummy
  dummySimulator.run(initObservation, initTargetTrajectories);
}

}  // namespace switched_model

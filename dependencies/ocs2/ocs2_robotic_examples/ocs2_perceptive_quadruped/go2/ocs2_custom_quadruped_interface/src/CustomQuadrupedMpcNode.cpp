//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_custom_quadruped_interface/CustomQuadrupedMpcNode.h"

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_custom_quadruped_interface/CustomSwingPlanningVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainPlaneVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainReceiver.h>

namespace switched_model {

void customQuadrupedMpcNode(const rclcpp::Node::SharedPtr &node, const CustomQuadrupedInterface& quadrupedInterface, std::unique_ptr<ocs2::MPC_BASE> mpcPtr) 
{
  const std::string robotName = "go2";

  auto solverModules = quadrupedInterface.getSynchronizedModules();

  // Gait
  auto gaitReceiver =
      std::make_shared<GaitReceiver>(node, quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
  solverModules.push_back(gaitReceiver);

  // Terrain Receiver
  auto terrainReceiver = std::make_shared<CustomTerrainReceiverSynchronizedModule>(
      quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), node);
  solverModules.push_back(terrainReceiver);

  // Terrain plane visualization
  auto terrainVisualizer = std::make_shared<CustomTerrainPlaneVisualizerSynchronizedModule>(
      quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node);
  solverModules.push_back(terrainVisualizer);

  // Swing planner
  auto swingPlanningVisualizer = std::make_shared<CustomSwingPlanningVisualizer>(
      quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node);
  solverModules.push_back(swingPlanningVisualizer);

  // reference manager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, quadrupedInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node);
  mpcPtr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // MPC
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);

  // launch MPC nodes
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(node);
}
}  // namespace switched_model

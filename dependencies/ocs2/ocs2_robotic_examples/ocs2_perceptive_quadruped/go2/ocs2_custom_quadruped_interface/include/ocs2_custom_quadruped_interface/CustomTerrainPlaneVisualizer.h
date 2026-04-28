//
// Created by rgrandia on 30.04.20.
//

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace switched_model {

class CustomTerrainPlaneVisualizer {
public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";  // Frame name all messages are published in
  double planeWidth_ = 1.5;
  double planeLength_ = 1.5;
  double planeThickness_ = 0.005;
  double planeAlpha_ = 0.5;

  explicit CustomTerrainPlaneVisualizer(const rclcpp::Node::SharedPtr& node);

  void update(scalar_t time, const TerrainPlane& terrainPlane);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      terrainPublisher_;
};

class CustomTerrainPlaneVisualizerSynchronizedModule : public ocs2::SolverSynchronizedModule {
 public:
  CustomTerrainPlaneVisualizerSynchronizedModule(
      const SwingTrajectoryPlanner& swingTrajectoryPlanner,
      const rclcpp::Node::SharedPtr& node);

  void preSolverRun(
      scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
      const ocs2::ReferenceManagerInterface& referenceManager) override{};

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override;

 private:
  const SwingTrajectoryPlanner* swingTrajectoryPlanner_;
  CustomTerrainPlaneVisualizer planeVisualizer_;
};

}  // namespace switched_model

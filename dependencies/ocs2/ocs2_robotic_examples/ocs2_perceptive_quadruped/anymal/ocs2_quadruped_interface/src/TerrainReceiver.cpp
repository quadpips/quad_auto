//
// Created by rgrandia on 28.09.20.
//

#include "ocs2_quadruped_interface/TerrainReceiver.h"

namespace switched_model {

TerrainReceiverSynchronizedModule::TerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel>& terrainModel,
                                                                                  const rclcpp::Node::SharedPtr &node,
                                                                                  const bool & vis)
    : terrainModelPtr_(&terrainModel), segmentedPlanesRos_(new switched_model::SegmentedPlanesTerrainModelRos(node, vis)) 
{
  // RCLCPP_INFO_STREAM(node->get_logger(), "[TerrainReceiverSynchronizedModule] Constructor called.");
}

void TerrainReceiverSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                                     const ocs2::ReferenceManagerInterface& referenceManager)
{
  if (auto newTerrain = segmentedPlanesRos_->getTerrainModel()) {
    terrainModelPtr_->reset(std::move(newTerrain));
    segmentedPlanesRos_->publish();
  }
}

}  // namespace switched_model
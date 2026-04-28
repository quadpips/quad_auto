//
// Created by rgrandia on 28.09.20.
//

#include "ocs2_custom_quadruped_interface/CustomTerrainReceiver.h"

namespace switched_model {

CustomTerrainReceiverSynchronizedModule::CustomTerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel>& terrainModel,
                                                                                  const rclcpp::Node::SharedPtr &node,
                                                                                  const bool & vis)
    : terrainModelPtr_(&terrainModel), segmentedPlanesRos_(new switched_model::SegmentedPlanesTerrainModelRos(node, vis)) 
{
  // RCLCPP_INFO_STREAM(node->get_logger(), "[CustomTerrainReceiverSynchronizedModule] Constructor called.");
}

void CustomTerrainReceiverSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                                     const ocs2::ReferenceManagerInterface& referenceManager)
{
  if (auto newTerrain = segmentedPlanesRos_->getTerrainModel()) {
    terrainModelPtr_->reset(std::move(newTerrain));
    segmentedPlanesRos_->publish();
  }
}

}  // namespace switched_model
/*! @file CheaterPositionVelocityEstimator.cpp
 *  @brief Class for maintaining the correct position and velocity
 *          when running in simulation.
 */

#include "go2_estimators/estimators/CheaterPositionVelocityEstimator.h"

namespace legged_software {
namespace go2_estimators {
  
/*!
 * Copy ground-truth state into state estimate
 */

void CheaterPositionVelocityEstimator::run() 
{
  // Check if we are not using the world frame
  if(!this->_stateEstimatorData->cheaterState->use_world_frame) 
  {
    // Save body-frame velocity and convert to world frame
    this->_stateEstimatorData->result.vBody = this->_stateEstimatorData->cheaterState->vBody;
    this->_stateEstimatorData->result.vWorld = this->_stateEstimatorData->result.rBody.transpose() * this->_stateEstimatorData->cheaterState->vBody;
  } 
  
  // Else we are using world-frame reference
  else 
  {
    // Save world-frame velocity and convert to body frame
    this->_stateEstimatorData->result.vWorld = this->_stateEstimatorData->cheaterState->vWorld;
    this->_stateEstimatorData->result.vBody = this->_stateEstimatorData->result.rBody * this->_stateEstimatorData->cheaterState->vWorld;
  }

  // Save position
  this->_stateEstimatorData->result.position = this->_stateEstimatorData->cheaterState->position;
}


} // namespace go2_estimators
} // namespace legged_software

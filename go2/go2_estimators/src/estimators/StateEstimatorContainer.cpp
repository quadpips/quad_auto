/*!
 * @file StateEstimator.cpp
 * @brief Definitions for handling state estimators and their data
 */

#include "go2_estimators/estimators/StateEstimatorContainer.h"

namespace legged_software {
namespace go2_estimators {

/*
*   Constructor
*/

StateEstimatorContainer::StateEstimatorContainer(
  CheaterState* cheaterState,
  ImuData* imuData,
  LimbData** limbData,
  QuadrupedParameters* parameters
)
{
  // Save initialization arguments
  _data.cheaterState = cheaterState;
  _data.imuData = imuData;
  _data.limbData = limbData;
  _data.parameters = parameters;
}

/*
*   Run all state estimators in container
*/

void StateEstimatorContainer::run()
{
  for (auto estimator : _estimators) 
  {
    estimator->run();
  }
}

} // namespace go2_estimators
} // namespace legged_software

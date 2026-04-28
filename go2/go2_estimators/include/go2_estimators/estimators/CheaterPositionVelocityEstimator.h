/*! @file CheaterPositionVelocityEstimator.h
 *  @brief Class for maintaining the correct position and velocity
 *          when running in simulation.
 */

#pragma once

#include <go2_estimators/estimators/StateEstimatorContainer.h>

namespace legged_software {
namespace go2_estimators {

using go2_estimators::GenericEstimator;

class CheaterPositionVelocityEstimator : public GenericEstimator {
public:
  virtual void run();
  virtual void setup() {}
};

} // namespace go2_estimators
} // namespace legged_software

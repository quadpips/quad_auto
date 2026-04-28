/*! @file OrientationObserver.h
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */
#pragma once

#include "go2_estimators/estimators/StateEstimatorContainer.h"

namespace legged_software {
namespace go2_estimators {

/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */

class CheaterOCS2OrientationObserver : public GenericEstimator 
{
 public:
  virtual void run();
  virtual void setup() {}

  protected:
    double lastYaw_ = 0.0;
};

/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */

class CheaterOrientationObserver : public GenericEstimator {
 public:
  virtual void run();
  virtual void setup() {}
};

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */

class VectorNavOCS2OrientationObserver : public GenericEstimator 
{
 public:
  virtual void run();
  virtual void setup() {}
  
 protected:
  bool _b_first_visit = true;
  Eigen::Quaterniond _ori_ini_inv = Eigen::Quaterniond::Identity();
  double lastYaw_ = 0.0;
};

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */

class VectorNavOrientationObserver : public GenericEstimator 
{
 public:
  virtual void run();
  virtual void setup() {}
  
 protected:
  bool _b_first_visit = true;
  Eigen::Quaterniond _ori_ini_inv = Eigen::Quaterniond::Identity();
};

} // namespace go2_estimators
} // namespace legged_software

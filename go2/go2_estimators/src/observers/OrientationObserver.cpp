/*! @file OrientationObserver.cpp
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

#include "go2_estimators/observers/OrientationObserver.h"

namespace legged_software {
namespace go2_estimators {

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) by copying from cheater state data
 */

void CheaterOCS2OrientationObserver::run() 
{
  this->_stateEstimatorData->result.orientation =
      this->_stateEstimatorData->cheaterState->orientation;
  this->_stateEstimatorData->result.rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData->result.orientation);
  this->_stateEstimatorData->result.rpy =
      ori::ocs2quatToRPY(this->_stateEstimatorData->result.orientation, lastYaw_); 
  lastYaw_ = this->_stateEstimatorData->result.rpy[2];

  if (!this->_stateEstimatorData->cheaterState->use_world_frame) 
  {
    this->_stateEstimatorData->result.omegaBody =
      this->_stateEstimatorData->cheaterState->omegaBody;
    this->_stateEstimatorData->result.omegaWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.omegaBody;
    this->_stateEstimatorData->result.aBody =
      this->_stateEstimatorData->cheaterState->aBody;
    this->_stateEstimatorData->result.aWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.aBody;
  } else 
  {
    this->_stateEstimatorData->result.omegaWorld =
      this->_stateEstimatorData->cheaterState->omegaWorld;
    this->_stateEstimatorData->result.omegaBody =
      this->_stateEstimatorData->result.rBody * this->_stateEstimatorData->cheaterState->omegaWorld;
    this->_stateEstimatorData->result.aWorld =
      this->_stateEstimatorData->cheaterState->aWorld;
    this->_stateEstimatorData->result.aBody =
      this->_stateEstimatorData->result.rBody *
      this->_stateEstimatorData->result.aWorld;
  }   
}

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) by copying from cheater state data
 */

void CheaterOrientationObserver::run() {
  this->_stateEstimatorData->result.orientation =
      this->_stateEstimatorData->cheaterState->orientation;
  this->_stateEstimatorData->result.rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData->result.orientation);
  this->_stateEstimatorData->result.rpy =
      ori::quatToRPY(this->_stateEstimatorData->result.orientation);

  if (!this->_stateEstimatorData->cheaterState->use_world_frame) 
  {
    this->_stateEstimatorData->result.omegaBody =
      this->_stateEstimatorData->cheaterState->omegaBody;
    this->_stateEstimatorData->result.omegaWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.omegaBody;
    this->_stateEstimatorData->result.aBody =
      this->_stateEstimatorData->cheaterState->aBody;
    this->_stateEstimatorData->result.aWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.aBody;
  } else 
  {
    this->_stateEstimatorData->result.omegaWorld =
      this->_stateEstimatorData->cheaterState->omegaWorld;
    this->_stateEstimatorData->result.omegaBody =
      this->_stateEstimatorData->result.rBody * this->_stateEstimatorData->cheaterState->omegaWorld;
    this->_stateEstimatorData->result.aWorld =
      this->_stateEstimatorData->cheaterState->aWorld;
    this->_stateEstimatorData->result.aBody =
      this->_stateEstimatorData->result.rBody *
      this->_stateEstimatorData->result.aWorld;
  }   
  
}

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */

void VectorNavOCS2OrientationObserver::run() 
{
  this->_stateEstimatorData->result.orientation = this->_stateEstimatorData->imuData->quat;


  if (_b_first_visit)
  {
    // std::cout << "[VectorNavOCS2OrientationObserver] first visit" << std::endl;
    // std::cout << "[VectorNavOCS2OrientationObserver] orientation: " 
              // << this->_stateEstimatorData->result.orientation.transpose() << std::endl;
    // std::cout << "[VectorNavOCS2OrientationObserver] lastYaw_: " << lastYaw_ << std::endl;
    Eigen::Vector3d rpy_ini = ori::ocs2quatToRPY(this->_stateEstimatorData->result.orientation, lastYaw_); 
    lastYaw_ = this->_stateEstimatorData->result.rpy[2];

    // std::cout << "[VectorNavOCS2OrientationObserver] Initial rpy: " << rpy_ini.transpose() << std::endl;

    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = rpyToQuat(-rpy_ini);
    _b_first_visit = false;
  }

  this->_stateEstimatorData->result.orientation = 
    ori::quatProduct(_ori_ini_inv, this->_stateEstimatorData->result.orientation);

  // std::cout << "[VectorNavOCS2OrientationObserver] Orientation: " << this->_stateEstimatorData->result.orientation.transpose() << std::endl;

  // Orientation
  this->_stateEstimatorData->result.rpy = ori::ocs2quatToRPY(this->_stateEstimatorData->result.orientation, lastYaw_);
  lastYaw_ = this->_stateEstimatorData->result.rpy[2];

  // std::cout << "[VectorNavOCS2OrientationObserver] rpy: " << this->_stateEstimatorData->result.rpy.transpose() << std::endl;
  // std::cout << "[VectorNavOCS2OrientationObserver] lastYaw_: " << lastYaw_ << std::endl;

  this->_stateEstimatorData->result.rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData->result.orientation);

  // std::cout << "[VectorNavOCS2OrientationObserver] rBody: " << this->_stateEstimatorData->result.rBody << std::endl;

  // Angular velocity
  this->_stateEstimatorData->result.omegaBody =
      this->_stateEstimatorData->imuData->gyro;

  // std::cout << "[VectorNavOCS2OrientationObserver] omegaBody: " << this->_stateEstimatorData->result.omegaBody << std::endl;

  this->_stateEstimatorData->result.omegaWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.omegaBody;

  // std::cout << "[VectorNavOCS2OrientationObserver] omegaWorld: " << this->_stateEstimatorData->result.omegaWorld << std::endl;

  // Linear Acceleration
  this->_stateEstimatorData->result.aBody =
      this->_stateEstimatorData->imuData->accelerometer;

  // std::cout << "[VectorNavOCS2OrientationObserver] aBody: " << this->_stateEstimatorData->result.aBody << std::endl;

  this->_stateEstimatorData->result.aWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.aBody;

  // std::cout << "[VectorNavOCS2OrientationObserver] aWorld: " << this->_stateEstimatorData->result.aWorld << std::endl;
}

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */

void VectorNavOrientationObserver::run() 
{
  // std::cerr << "[VectorNavOrientationObserver] run() " << std::endl;
  this->_stateEstimatorData->result.orientation = this->_stateEstimatorData->imuData->quat;

  // std::cerr << "[VectorNavOrientationObserver] orientation (one): " 
  //           << this->_stateEstimatorData->result.orientation.transpose() << std::endl;

  if (_b_first_visit)
  {
    Eigen::Vector3d rpy_ini = ori::quatToRPY(this->_stateEstimatorData->result.orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = rpyToQuat(-rpy_ini);
    _b_first_visit = false;
  }

  this->_stateEstimatorData->result.orientation = 
    ori::quatProduct(_ori_ini_inv, this->_stateEstimatorData->result.orientation);

  // std::cerr << "[VectorNavOrientationObserver] orientation (two): " 
  //           << this->_stateEstimatorData->result.orientation.transpose() << std::endl;

  this->_stateEstimatorData->result.rpy =
      ori::quatToRPY(this->_stateEstimatorData->result.orientation);

  // std::cerr << "[VectorNavOrientationObserver] rpy: " 
  //           << this->_stateEstimatorData->result.rpy.transpose() << std::endl;

  this->_stateEstimatorData->result.rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData->result.orientation);

  // std::cerr << "[VectorNavOrientationObserver] rBody: " 
            // << this->_stateEstimatorData->result.rBody << std::endl;

  this->_stateEstimatorData->result.omegaBody =
      this->_stateEstimatorData->imuData->gyro;

  // std::cerr << "[VectorNavOrientationObserver] omegaBody: " 
            // << this->_stateEstimatorData->result.omegaBody.transpose() << std::endl;

  this->_stateEstimatorData->result.omegaWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.omegaBody;

  // std::cerr << "[VectorNavOrientationObserver] omegaWorld: " 
            // << this->_stateEstimatorData->result.omegaWorld.transpose() << std::endl;

  this->_stateEstimatorData->result.aBody =
      this->_stateEstimatorData->imuData->accelerometer;

  // std::cerr << "[VectorNavOrientationObserver] aBody: " 
            // << this->_stateEstimatorData->result.aBody.transpose() << std::endl;

  this->_stateEstimatorData->result.aWorld =
      this->_stateEstimatorData->result.rBody.transpose() *
      this->_stateEstimatorData->result.aBody;

  // std::cerr << "[VectorNavOrientationObserver] aWorld: " 
  //           << this->_stateEstimatorData->result.aWorld.transpose() << std::endl;
}

} // namespace go2_estimators
} // namespace legged_software

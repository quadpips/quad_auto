/*! @file IMUTypes.h
 *  @brief Data from IMUs
 */

#pragma once

namespace legged_software {
namespace go2_estimators {

/*!
 * IMU data structure
 */
struct ImuData 
{
  Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity(); // w, x, y, z
  // todo is there status for the vectornav?
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
struct CheaterState 
{
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // w, x, y, z
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d omegaBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d vBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d omegaWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d vWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d aBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d aWorld = Eigen::Vector3d::Zero();
  bool use_world_frame = false;
};

} // namespace go2_estimators
} // namespace legged_software

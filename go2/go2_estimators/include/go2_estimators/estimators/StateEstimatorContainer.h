/*!
 * @file StateEstimator.h
 * @brief Definitions for handling state estimators and their data
 */

#pragma once

#include <iostream>

#include <legged_utils/math/orientation_tools.h>
#include "legged_common/control/ctrl_utils/QuadrupedParameters.h"
#include "legged_common/control/ctrl_utils/LimbData.hpp"
#include "go2_estimators/estimators/IMUTypes.h"

#include "rclcpp/rclcpp.hpp"

using namespace legged_software::ori;
using namespace legged_software::legged_common;

namespace legged_software {
namespace go2_estimators {

/*!
 * Result of state estimation.
 */

struct StateEstimate 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d contactEstimate = Eigen::Vector4d::Zero();
  size_t gaitMode = 0; // aligned with OCS2 convention

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d vBody = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // w, x, y, z
  Eigen::Vector3d omegaBody = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rBody = Eigen::Matrix3d::Identity();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();

  Eigen::Vector3d omegaWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d vWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d aBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d aWorld = Eigen::Vector3d::Zero();
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */

struct StateEstimatorData 
{
  StateEstimate result;  // where to write the output to

  // Contact information
  Eigen::Vector4d footHeights = Eigen::Vector4d::Zero();
  Eigen::VectorXd contactForces = Eigen::VectorXd::Zero(12);

  // Sensor information
  const ImuData* imuData;
  const CheaterState* cheaterState;
  LimbData* const * limbData;

  // Robot information
  const QuadrupedParameters* parameters;
};

/*!
 * All Estimators should inherit from this class
 */

class GenericEstimator 
{
  public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData * data) { _stateEstimatorData = data; }

    virtual ~GenericEstimator() = default;
    StateEstimatorData * _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */

class StateEstimatorContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor
   */
  StateEstimatorContainer(
    CheaterState* cheaterState,
    ImuData* imuData,
    LimbData** limbData,
    QuadrupedParameters* parameters
  );

  /*!
   * Run all estimators
   */
  void run();

  /*!
   * Get the result
   */
  const StateEstimate& getResult() { return _data.result; }

  /*!
   * Set the contact estimate
   */
  void setContactEstimate(const Eigen::Vector4d& contactEstimate) {
    _data.result.contactEstimate = contactEstimate;
  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() 
  {
    auto* estimator = new EstimatorToAdd();
    estimator->setData(&_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator(const rclcpp::Node::SharedPtr & node) 
  {
    auto* estimator = new EstimatorToAdd(node);
    estimator->setData(&_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
  /*!
   * Remove all estimators
   */
  void removeAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  ~StateEstimatorContainer() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

private:
  StateEstimatorData _data;
  std::vector<GenericEstimator*> _estimators;
};

} // namespace go2_estimators
} // namespace legged_software

/*! @file InEKFEstimator.h
 *  @brief Container for InEKF State Estimation Algorithm
 */

#pragma once

// #include <pinocchio/multibody/data.hpp>
// #include <pinocchio/multibody/model.hpp>

// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/parsers/urdf.hpp>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

#include <go2_estimators/estimators/InEKF.h>
#include <go2_estimators/estimators/StateEstimatorContainer.h>

namespace legged_software {
namespace go2_estimators {

using go2_estimators::GenericEstimator;

/*!
 * Position and velocity estimator based on Invariant Extended Kalman Filter (InEKF)
 */

class InEKFEstimator : public GenericEstimator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    InEKFEstimator(const rclcpp::Node::SharedPtr & node);

    // GenericEstimator pure virtual functions
    virtual void run();
    virtual void setup();

  private:

    void propagate();
    void setContacts();
    void correctKinematics();

    Eigen::Vector3d lastAngularVelocity_;
    Eigen::Vector3d lastLinearAcceleration_;

    rclcpp::Node::SharedPtr node_;

    // pinocchio::Model model_;
    // pinocchio::Data data_;
    
    std::shared_ptr<switched_model::CustomQuadrupedInterface> legged_interface_;

    go2_estimators::InEKF inekf_;

    std::vector<size_t> legIndices_;

    bool _b_first_visit = true;
    Eigen::Quaterniond _ori_ini_inv = Eigen::Quaterniond::Identity();
    double lastYaw_ = 0.0;

    bool rectifyOrientation_ = true;
};

} // namespace go2_estimators
} // namespace legged_software
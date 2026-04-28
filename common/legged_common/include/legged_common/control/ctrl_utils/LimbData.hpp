#pragma once

#include <Eigen/Dense>

namespace legged_software {
namespace legged_common {

class LimbData{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LimbData(){ }
    virtual ~LimbData(){}
    virtual void zero() = 0;

    Eigen::Vector3d tauEstimate = Eigen::Vector3d::Zero(); // Estimated joint torques
    Eigen::Vector3d q = Eigen::Vector3d::Zero(); // Joint positions
    Eigen::Vector3d qd = Eigen::Vector3d::Zero(); // Joint velocities
};

} // namespace legged_common
} // namespace legged_software

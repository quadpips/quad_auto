/*! @file FloatingBaseState.h
 *  @brief Implementation of Rigid Body Floating Base model data structure
 *
 * This class stores the kinematic tree described in "Rigid Body Dynamics
 * Algorithms" by Featherstone (download from
 * https://www.springer.com/us/book/9780387743141 on MIT internet)
 *
 * The tree includes an additional "rotor" body for each body.  This rotor is
 * fixed to the parent body and has a gearing constraint.  This is efficiently
 * included using a technique similar to what is described in Chapter 12 of
 * "Robot and Multibody Dynamics" by Jain.  Note that this implementation is
 * highly specific to the case of a single rotating rotor per rigid body. Rotors
 * have the same joint type as their body, but with an additional gear ratio
 * multiplier applied to the motion subspace. The rotors associated with the
 * floating base don't do anything.
 */

#pragma once

#include <string>
#include <vector>

#include <legged_utils/math/orientation_tools.h>

// #include <eigen3/Eigen/StdVector>

namespace legged_software {
namespace legged_common {

using std::vector;

/*!
 * The state of a floating base model (base and joints)
 */
class FBModelState 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    FBModelState()
    {
      bodyOrientation = Eigen::Quaterniond::Identity();
      
      bodyPosition.setZero();
    }
    
    void operator = (const FBModelState& state)
    {
      this->bodyOrientation = state.bodyOrientation;
      this->bodyPosition = state.bodyPosition;
      this->q = state.q;
      this->qd = state.qd;
    }

    Eigen::Quaterniond bodyOrientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bodyPosition = Eigen::Vector3d::Zero();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(12);

    /*!
     * Print the position of the body
     */
    void print() const 
    {
      printf("body position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1], bodyPosition[2]);
      printf("body orientation: %.3f %.3f %.3f %.3f\n", bodyOrientation.w(), bodyOrientation.x(), bodyOrientation.y(), bodyOrientation.z());
      printf("q: \t");

      for (int i(0); i<q.rows(); ++i)
      {
        printf("%.3f, \t", q[i]);
      }
      printf("\n");
    }
};

} // namespace legged_common
} // namespace legged_software
/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters which are shared between all robot controllers
 *  Currently there are some userParameters that are specific to the MIT controllers here,
 *  but these will be moved in the future
 */

#pragma once

namespace legged_software {
namespace legged_common {

// using legged_utils::ControlParameters;

/*!
 * ControlParameters shared between all robot controllers
 */
class QuadrupedParameters {
 public:

  /*!
   * Construct RobotControlParameters
   */
  QuadrupedParameters()
  {}

  int control_mode = 0;  /**< Control mode (0: normal, 1: stand, 2: sit, etc.) */
  int cheater_mode = 0;  /**< Whether to use cheater mode for state estimation (1: enabled, 0: disabled) */
  double controller_dt = 0.002;  /**< Control loop time step */
  double estimator_dt = 0.002;  /**< State estimator time step */

};

} // namespace legged_common
} // namespace legged_software

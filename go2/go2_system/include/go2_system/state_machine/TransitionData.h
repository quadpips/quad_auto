/*============================= FSM State =============================*/
#pragma once

namespace legged_software {
namespace go2_system {

/**
 * Struct of relevant data that can be used during transition to pass
 * data between states.
 */
struct TransitionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TransitionData() { zero(); }

  // Zero out all of the data
  void zero() 
  {
    // Flag to mark when transition is done
    done = false;

  }

  // Flag to mark when transition is done
  bool done = false;

};

} // namespace go2_system
} // namespace legged_software

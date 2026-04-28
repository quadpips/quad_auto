#pragma once

#include <go2_system/state_machine/FSM_State.h>

namespace legged_software {
namespace go2_system {

/**
 *
 */
class FSM_State_Passive : public FSM_State {
 public:
  FSM_State_Passive(ControlFSMData* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData testTransition();

 private:
  // Keep track of the control iterations
  int iter = 0;
};

} // namespace go2_system
} // namespace legged_software

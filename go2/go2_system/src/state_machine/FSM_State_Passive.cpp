/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "go2_system/state_machine/FSM_State_Passive.h"

namespace legged_software {
namespace go2_system {

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSM_State_Passive::FSM_State_Passive(ControlFSMData* _controlFSMData)
    : FSM_State(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE") 
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;
}

void FSM_State_Passive::onEnter() 
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_Passive::run() 
{
  // Do nothing, all commands should begin as zeros
  for (int i = 0; i < num_leg; ++i) 
  {
    // this->_data->_legController->commands[i].zero();
    this->_data->_legController->edampCommand(3.0); // RobotType::GO2,  
  }
  testTransition();
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
TransitionData FSM_State_Passive::testTransition() 
{
  this->transitionData.done = true;
  return this->transitionData;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_Passive::checkTransition() 
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->control_mode) 
  {
    case FSM_StateName::PASSIVE:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << (int)FSM_StateName::PASSIVE << " to "
                << this->_data->userParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
TransitionData FSM_State_Passive::transition() 
{
  // Finish Transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
void FSM_State_Passive::onExit() 
{
  // Nothing to clean up when exiting
}

} // namespace go2_system
} // namespace legged_software

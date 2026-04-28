/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "go2_system/state_machine/ControlFSM.h"

// FSM States
#include "go2_system/state_machine/FSM_State.h"
#include "go2_system/state_machine/FSM_State_Passive.h"
#include "go2_system/state_machine/FSM_State_RecoveryStand.h"
#include "go2_system/state_machine/FSM_State_PerceptiveLocomotionOCS2.h"

namespace legged_software {
namespace go2_system {


/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _highcmd gets the high-level commands
 * @param userParameters passes in the control parameters from the GUI
 */
ControlFSM::ControlFSM(const rclcpp::Node::SharedPtr & node,
                          StateEstimatorContainer* _stateEstimator,
                          LegController* _legController,
                          HighCmdCustom* _highcmd,
                          QuadrupedParameters* userParameters)
{
  // Add the pointers to the ControlFSMData struct
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data._highcmd = _highcmd;
  data.userParameters = userParameters;

  // Initialize and add all of the FSM States to the state list
  _state_list.resize(FSM_StateName::NUM_STATE);
  _state_list[FSM_StateName::INVALID] = nullptr;
  _state_list[FSM_StateName::PASSIVE] = new FSM_State_Passive(&data);                  // 0
  _state_list[FSM_StateName::PERCEPTIVELOCOMOTIONOCS2] = new FSM_State_PerceptiveLocomotionOCS2(node, &data); // 4
  _state_list[FSM_StateName::RECOVERY_STAND] = new FSM_State_RecoveryStand(&data);     // 5
  
  safetyChecker = new SafetyChecker(&data);

  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
void ControlFSM::initialize() 
{
  // ROS_INFO_STREAM("[ControlFSM::initialize()] started");

  // Initialize a new FSM State with the control data
  currentState = _state_list[FSM_StateName::PASSIVE];

  // ROS_INFO_STREAM("[ControlFSM::initialize()]     1");

  // Enter the new current state cleanly
  currentState->onEnter();

  // ROS_INFO_STREAM("[ControlFSM::initialize()]     2");

  // Initialize to not be in transition
  nextState = currentState;

  // ROS_INFO_STREAM("[ControlFSM::initialize()]     3");

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;

  // ROS_INFO_STREAM("[ControlFSM::initialize()] finished");
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
void ControlFSM::runFSM() 
{
  // ROS_INFO_STREAM("[ControlFSM::runFSM()] started");

  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

  // Control logic for high level command
  int auto_mode = data._highcmd->mode;
  if (auto_mode == 0) 
  {
    data.userParameters->control_mode = FSM_StateName::PASSIVE;
  } else if (auto_mode == 1) 
  {
    data.userParameters->control_mode = FSM_StateName::RECOVERY_STAND;
  } else if (auto_mode == 4) 
  {
    data.userParameters->control_mode = FSM_StateName::PERCEPTIVELOCOMOTIONOCS2;
  } else 
  {
    std::cout << "[CONTROL FSM] control mode not developed yet" << std::endl;
  }

  // ROS_INFO_STREAM("[ControlFSM::runFSM()]     data.userParameters->control_mode: " << data.userParameters->control_mode);

  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != FSM_OperatingMode::ESTOP) 
  {
    // ROS_INFO_STREAM("[ControlFSM::runFSM()]     operatingMode != FSM_OperatingMode::ESTOP");

    // Run normal controls if no transition is detected
    if (operatingMode == FSM_OperatingMode::NORMAL) 
    {

      // ROS_INFO_STREAM("[ControlFSM::runFSM()]     operatingMode == FSM_OperatingMode::NORMAL");

      // Check the current state for any transition
      nextStateName = currentState->checkTransition();

      // Detect a commanded transition
      if (nextStateName != currentState->stateName) 
      {
        // ROS_INFO_STREAM("[ControlFSM::runFSM()]     operatingMode == FSM_OperatingMode::TRANSITIONING");

        // Set the FSM operating mode to transitioning
        operatingMode = FSM_OperatingMode::TRANSITIONING;

        // Get the next FSM State by name
        nextState = _state_list[nextStateName];

        // Print transition initialized info
        //printInfo(1);

      } else 
      {
        // Run the iteration for the current state normally
        currentState->run();
      }
    }

    // Run the transition code while transition is occuring
    if (operatingMode == FSM_OperatingMode::TRANSITIONING) 
    {
      // ROS_INFO_STREAM("[ControlFSM::runFSM()]     operatingMode == FSM_OperatingMode::TRANSITIONING");

      transitionData = currentState->transition();

      // Check the robot state for safe operation
      safetyPostCheck();

      // Run the state transition
      if (transitionData.done) {
        // Exit the current state cleanly
        currentState->onExit();

        // Print finalizing transition info
        //printInfo(2);

        // Complete the transition
        currentState = nextState;

        // Enter the new current state cleanly
        currentState->onEnter();

        // Return the FSM to normal operation mode
        operatingMode = FSM_OperatingMode::NORMAL;
      }
    } else 
    {
      // ROS_INFO_STREAM("[ControlFSM::runFSM()]     safetyPostCheck()");
      // Check the robot state for safe operation
      safetyPostCheck();
    }

  } else 
  { // if ESTOP
    // ROS_INFO_STREAM("[ControlFSM::runFSM()]     operatingMode == FSM_OperatingMode::ESTOP");
  
    currentState = _state_list[FSM_StateName::PASSIVE];
    currentState->onEnter();
    nextStateName = currentState->stateName;
  }

  // Print the current state of the FSM
  printInfo(0);

  // Increase the iteration counter
  iter++;

  return;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
FSM_OperatingMode ControlFSM::safetyPreCheck() 
{
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation && data.userParameters->control_mode != FSM_StateName::RECOVERY_STAND) 
  {
    if (!safetyChecker->checkSafeOrientation()) 
    {
      operatingMode = FSM_OperatingMode::ESTOP;
      // data.userParameters->control_mode = FSM_StateName::RECOVERY_STAND;
      std::cout << "[CONTROL_FSM] broken: Orientation Safety Check FAIL" << std::endl;
    }
  }

  if (currentState->checkSafeHeight && data.userParameters->control_mode != FSM_StateName::RECOVERY_STAND)  
  {
    if (!safetyChecker->checkSafeHeight()) 
    {
      operatingMode = FSM_OperatingMode::ESTOP;
      // data.userParameters->control_mode = FSM_StateName::RECOVERY_STAND;
      std::cout << "[CONTROL_FSM] broken: Height Safety Check FAIL" << std::endl;
    }    
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
FSM_OperatingMode ControlFSM::safetyPostCheck() 
{

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
void ControlFSM::printInfo(int opt) 
{
  switch (opt) 
  {
    case 0:  // Normal printing case at regular intervals
      // Increment printing iteration
      printIter++;

      // Print at commanded frequency
      if (printIter == printNum) 
      {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout
            << "---------------------------------------------------------\n";
        std::cout << "Iteration: " << iter << "\n";
        if (operatingMode == FSM_OperatingMode::NORMAL) 
        {
          std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                    << "\n";

        } else if (operatingMode == FSM_OperatingMode::TRANSITIONING) 
        {
          std::cout << "Operating Mode: TRANSITIONING from "
                    << currentState->stateString << " to "
                    << nextState->stateString << "\n";

        } else if (operatingMode == FSM_OperatingMode::ESTOP) 
        {
          std::cout << "Operating Mode: ESTOP\n";
        }
        std::cout << std::endl;

        // Reset iteration counter
        printIter = 0;
      }

      break;

    case 1:  // Initializing FSM State transition
      std::cout << "[CONTROL FSM] Transition initialized from "
                << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;

    case 2:  // Finalizing FSM State transition
      std::cout << "[CONTROL FSM] Transition finalizing from "
                << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;
  }
}

} // namespace go2_system
} // namespace legged_software

#pragma once

#include <iostream>

// Checks the robot state and commands for safety
#include "go2_system/state_machine/SafetyChecker.h"
#include "go2_system/state_machine/TransitionData.h"

#include "rclcpp/rclcpp.hpp"

namespace legged_software {
namespace go2_system {
 
class FSM_State;
/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum FSM_StateName 
{
  INVALID = 0,
  PASSIVE = 1,
  PERCEPTIVELOCOMOTIONOCS2 = 4,
  RECOVERY_STAND = 5,
  NUM_STATE
};

constexpr size_t num_leg = 4;

/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode {NORMAL, TRANSITIONING, ESTOP, EDAMP };


/**
 * Control FSM handles the FSM states from a higher level
 */
class ControlFSM {
 public:
  // Initialize for Quadruped
  ControlFSM(const rclcpp::Node::SharedPtr & node,
              StateEstimatorContainer* _stateEstimator,
              LegController* _legController,
              HighCmdCustom* _highcmd,
              QuadrupedParameters* userParameters);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  // This will be removed and put into the SafetyChecker class
  FSM_OperatingMode safetyPreCheck();

  //
  FSM_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested
  FSM_State* getNextState(FSM_StateName stateName);

  // Prints the current FSM status
  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData data;

  // FSM state information
  std::vector<FSM_State *> _state_list;
  FSM_State* currentState;    // current FSM state
  FSM_State* nextState;       // next FSM state
  FSM_StateName nextStateName;   // next FSM state name

  // Checks all of the inputs and commands for safety
  SafetyChecker* safetyChecker;

  // Contains data relevant to state transitions
  TransitionData transitionData;

 private:
  // Operating mode of the FSM
  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 1000;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;  // make larger than printNum to not print

  int iter = 0;

};

} // namespace go2_system
} // namespace legged_software

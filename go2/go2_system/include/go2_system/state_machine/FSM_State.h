#pragma once

#include <stdio.h>

#include "go2_system/state_machine/ControlFSM.h"
#include "go2_system/state_machine/ControlFSMData.h"
#include "go2_system/state_machine/TransitionData.h"

namespace legged_software {
namespace go2_system {
  
/**
 *
 */
class FSM_State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  FSM_State(ControlFSMData* _controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}

  // Manages state specific transitions
  virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

  // Runs the transition behaviors and returns true when done transitioning
  virtual TransitionData transition() { return transitionData; }

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0; // {}

  void jointPDControl(const int & leg, const Eigen::Vector3d & qDes, const Eigen::Vector3d & qdDes);
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();
  bool locomotionSafe();

  // Holds all of the relevant control data
  ControlFSMData* _data;

  // FSM State info
  FSM_StateName stateName;      // enumerated name of the current state
  FSM_StateName nextStateName;  // enumerated name of the next state
  std::string stateString;      // state name string

  // Transition parameters
  TransitionData transitionData;

  // Pre controls safety checks
  bool checkSafeOrientation = false;  // check roll and pitch
  bool checkSafeHeight = false;      // check robot height

  // Leg controller command placeholders for the whole robot (3x4 matrices)
  Eigen::MatrixXd jointPositions = Eigen::MatrixXd::Zero(3, 4);           // joint angle positions
  Eigen::MatrixXd jointVelocities = Eigen::MatrixXd::Zero(3, 4);          // joint angular velocities

 private:
  // Create the cartesian P gain matrix
  Eigen::Matrix3d kpMat;

  // Create the cartesian D gain matrix
  Eigen::Matrix3d kdMat;
};

} // namespace go2_system
} // namespace legged_software
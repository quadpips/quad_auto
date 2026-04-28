#pragma once
#include <iostream>

// Contains all of the control related data
#include "go2_system/state_machine/ControlFSMData.h"

namespace legged_software {
namespace go2_system {

/**
 * The SafetyChecker handles the checks requested by the ControlFSM.
 */
class SafetyChecker {
 public:
  SafetyChecker(ControlFSMData* dataIn) : data(dataIn){};

  // Pre checks to make sure controls are safe to run
  bool checkSafeOrientation();  // robot's orientation is safe to control
  bool checkSafeHeight();       // robot's height is safe to control

  // Stores the data from the ControlFSM
  ControlFSMData* data;

 private:
};

} // namespace go2_system
} // namespace legged_software

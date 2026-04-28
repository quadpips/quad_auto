/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "go2_system/state_machine/SafetyChecker.h"

namespace legged_software {
namespace go2_system {

/**
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkSafeOrientation() 
{
  double maxRoll = M_PI / 2;   // 90 degrees
  double maxPitch = M_PI / 2;  // 90 degrees
  if (abs(data->_stateEstimator->getResult().rpy(0)) >= maxRoll || abs(data->_stateEstimator->getResult().rpy(1)) >= maxPitch) 
  {
    printf("Orientation safety check failed!\n");
    printf("Roll: %f, Pitch: %f\n",
           data->_stateEstimator->getResult().rpy(0),
           data->_stateEstimator->getResult().rpy(1));
    return false;
  } else 
  {
    return true;
  }
}

/**
 * @return true if safe height
 */
bool SafetyChecker::checkSafeHeight() 
{
  double minHeight = 0.0; // 0 cm
  if (data->_stateEstimator->getResult().position(2) <= minHeight) 
  {
    printf("Height safety check failed!\n");
    printf("Height: %f\n",
           data->_stateEstimator->getResult().position(2));
    return false;
  } else 
  {
    return true;
  }
}

} // namespace go2_system
} // namespace legged_software

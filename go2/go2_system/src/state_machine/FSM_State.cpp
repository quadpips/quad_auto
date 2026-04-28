/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "go2_system/state_machine/FSM_State.h"

namespace legged_software {
namespace go2_system {

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
FSM_State::FSM_State(ControlFSMData* _controlFSMData,
                        FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData),
      stateName(stateNameIn),
      stateString(stateStringIn) 
{
    transitionData.zero();

  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn
            << std::endl;
}

/**
 * Joint impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
void FSM_State::jointPDControl(const int & leg, const Eigen::Vector3d & qDes, const Eigen::Vector3d & qdDes) 
{
  kpMat <<    80.0, 0, 0,
              0, 80.0, 0, 
              0, 0, 80.0;
  kdMat <<    1, 0, 0, 
              0, 1, 0, 
              0, 0, 1;


  // std::cerr << "kpMat: " << kpMat << std::endl;
  // std::cerr << "kdMat: " << kdMat << std::endl;

  _data->_legController->commands[leg].kpJoint = kpMat;
  _data->_legController->commands[leg].kdJoint = kdMat;

  _data->_legController->commands[leg].qDes = qDes;
  _data->_legController->commands[leg].qdDes = qdDes;
}

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
void FSM_State::turnOnAllSafetyChecks() 
{
  // Pre controls safety checks
  checkSafeOrientation = true;  // check roll and pitch
  checkSafeHeight = true;       // check height
}

/**
 *
 */
void FSM_State::turnOffAllSafetyChecks() 
{
  // Pre controls safety checks
  checkSafeOrientation = false;  // check roll and pitch
}

bool FSM_State::locomotionSafe() 
{
  // auto &seResult = _data->_stateEstimator->getResult();

  // const T max_roll = 45;
  // const T max_pitch = 45;
  // const T max_vel = 15.0;

  // if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
  //   printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
  //   return false;
  // }

  // if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
  //   printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
  //   return false;
  // }

  // for (int leg = 0; leg < 4; leg++) {
  //   auto p_leg = _data->_legController->datas[leg]->p;
  //   if (p_leg[2] > 0) {
  //     printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
  //     return false;
  //   }

  //   if (std::fabs(p_leg[1]) > 0.30) {
  //     printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
  //     return false;
  //   }

  //   auto v_leg = _data->_legController->datas[leg]->v.norm();
  //   if (std::fabs(v_leg) > max_vel) {
  //     printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
  //     return false;
  //   }
  // }
  return true;
}

} // namespace go2_system
} // namespace legged_software

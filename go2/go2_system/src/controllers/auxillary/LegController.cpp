/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "go2_system/controllers/auxillary/LegController.h"

namespace legged_software {
namespace go2_system {
  
/*!
 * Zero the leg command so the leg will not output torque
 */
void LegControllerCommand::zero() 
{
  tauFeedForward.setZero();
  qDes.setZero();
  qdDes.setZero();
  kpJoint.setZero();
  kdJoint.setZero();
}

/*!
 * Zero the leg data
 */
void LegControllerData::zero() 
{
  this->q.setZero();
  this->qd.setZero();
  this->tauEstimate.setZero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
void LegController::zeroCommand() 
{
  for (auto& cmd : commands) 
  {
    cmd.zero();
  }
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
void LegController::edampCommand(double gain) // RobotType robot,  
{
  zeroCommand();

  for (int leg = 0; leg < 4; leg++) 
  {
    for (int axis = 0; axis < 3; axis++) 
    {
      commands[leg].kdJoint(axis, axis) = gain;
    }
  }

}

/*!
 * Update the "leg data" from a low-state message
 */
void LegController::updateData(const LowState * go2State) 
{
  for (int leg = 0; leg < 4; leg++) 
  {
    // q:
    datas[leg]->q(0) = go2State->motor_state[leg*3].q;
    datas[leg]->q(1) = go2State->motor_state[leg*3+1].q;
    datas[leg]->q(2) = go2State->motor_state[leg*3+2].q;

    // qd
    datas[leg]->qd(0) = go2State->motor_state[leg*3].dq;
    datas[leg]->qd(1) = go2State->motor_state[leg*3+1].dq;
    datas[leg]->qd(2) = go2State->motor_state[leg*3+2].dq;

    // Torque estimates
    datas[leg]->tauEstimate(0) = go2State->motor_state[leg*3].tau_est;
    datas[leg]->tauEstimate(1) = go2State->motor_state[leg*3+1].tau_est;
    datas[leg]->tauEstimate(2) = go2State->motor_state[leg*3+2].tau_est;
  }
}

/*!
 * Update the "leg command" for the low-command message
 */
void LegController::updateCommand(LowCmd * go2Command) 
{
    // std::cerr << "[LegController] updateCommand()]" << "\n";

    for (int leg = 0; leg < 4; leg++) 
    {
      // tauFF
      Eigen::Vector3d legTorque = commands[leg].tauFeedForward;

      // set command:
      go2Command->motor_cmd[leg*3].tau = legTorque(0);
      go2Command->motor_cmd[leg*3+1].tau = legTorque(1);
      go2Command->motor_cmd[leg*3+2].tau = legTorque(2);

      // joint space PD
      go2Command->motor_cmd[leg*3].kd = commands[leg].kdJoint(0, 0);
      go2Command->motor_cmd[leg*3+1].kd = commands[leg].kdJoint(1, 1);
      go2Command->motor_cmd[leg*3+2].kd = commands[leg].kdJoint(2, 2);

      go2Command->motor_cmd[leg*3].kp = commands[leg].kpJoint(0, 0);
      go2Command->motor_cmd[leg*3+1].kp = commands[leg].kpJoint(1, 1);
      go2Command->motor_cmd[leg*3+2].kp = commands[leg].kpJoint(2, 2);

      go2Command->motor_cmd[leg*3].q = commands[leg].qDes(0);
      go2Command->motor_cmd[leg*3+1].q = commands[leg].qDes(1);
      go2Command->motor_cmd[leg*3+2].q = commands[leg].qDes(2);

      go2Command->motor_cmd[leg*3].dq = commands[leg].qdDes(0);
      go2Command->motor_cmd[leg*3+1].dq = commands[leg].qdDes(1);
      go2Command->motor_cmd[leg*3+2].dq = commands[leg].qdDes(2);

      go2Command->motor_cmd[leg*3].mode = 0x01;
      go2Command->motor_cmd[leg*3+1].mode = 0x01;
      go2Command->motor_cmd[leg*3+2].mode = 0x01;

      // std::cerr << "  leg: " << leg << "\n";
      // std::cerr << "      tauFF: " << go2Command->motor_cmd[leg*3].tau << ", "
      //                             << go2Command->motor_cmd[leg*3+1].tau << ", "
      //                             << go2Command->motor_cmd[leg*3+2].tau << "\n";
      // std::cerr << "      kp: " << go2Command->motor_cmd[leg*3].kp << ", "
      //                             << go2Command->motor_cmd[leg*3+1].kp << ", "
      //                             << go2Command->motor_cmd[leg*3+2].kp << "\n";
      // std::cerr << "      kd: " << go2Command->motor_cmd[leg*3].kd << ", "
      //                             << go2Command->motor_cmd[leg*3+1].kd << ", "
      //                             << go2Command->motor_cmd[leg*3+2].kd << "\n";
      // std::cerr << "      q: " << go2Command->motor_cmd[leg*3].q << ", "
      //                             << go2Command->motor_cmd[leg*3+1].q << ", "
      //                             << go2Command->motor_cmd[leg*3+2].q << "\n";
      // std::cerr << "      dq: " << go2Command->motor_cmd[leg*3].dq << ", "
      //                             << go2Command->motor_cmd[leg*3+1].dq << ", "
      //                             << go2Command->motor_cmd[leg*3+2].dq << "\n";
      // std::cerr << "      mode: " << std::hex << go2Command->motor_cmd[leg*3].mode << ", "
      //                                         << go2Command->motor_cmd[leg*3+1].mode << ", "
      //                                         << go2Command->motor_cmd[leg*3+2].mode << std::dec << "\n";
    }
}

} // namespace go2_system
} // namespace legged_software



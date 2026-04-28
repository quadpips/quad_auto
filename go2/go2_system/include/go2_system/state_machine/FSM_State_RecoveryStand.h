#pragma once

#include "go2_system/state_machine/FSM_State.h"

namespace legged_software {
namespace go2_system {

/**
 *
 */
class FSM_State_RecoveryStand : public FSM_State {
 public:
  FSM_State_RecoveryStand(ControlFSMData* _controlFSMData);

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
  int _motion_start_iter = 0;

  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;
  static constexpr int RollOver = 2;

  unsigned long long _state_iter;
  int _flag = FoldLegs;
  bool roll_left = true; //Keeps track of last rolling direction

  // JPos
  Eigen::Vector3d fold_jpos[4] = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  Eigen::Vector3d stand_jpos[4] = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  Eigen::Vector3d rolling_jpos[4] = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  Eigen::Vector3d initial_jpos[4] = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  Eigen::Vector3d zero_vec3 = Eigen::Vector3d::Zero();

  // 0.5 kHz
  const int rollover_ramp_iter = 150;
  const int rollover_settle_iter = 150;

  const int fold_ramp_iter = 400;
  const int fold_settle_iter = 700;

  const int standup_ramp_iter = 500;
  const int standup_settle_iter = 250;

  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);

  bool _UpsideDown();
  void _AssignRollDirection();
  void _SetJPosInterPts(const size_t & curr_iter, const size_t & max_iter, const int & leg, 
                        const Eigen::Vector3d & ini, const Eigen::Vector3d & fin);

};

} // namespace go2_system
} // namespace legged_software

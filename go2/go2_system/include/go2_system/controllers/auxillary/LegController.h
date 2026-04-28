/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#pragma once

#include <legged_common/control/ctrl_utils/LimbData.hpp>

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace legged_software {
namespace go2_system {

using unitree_go::msg::LowCmd;
using unitree_go::msg::LowState;
using legged_common::LimbData;

/*!
 * Data sent from the control algorithm to the legs.
 */
struct LegControllerCommand 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Eigen::Vector3d tauFeedForward = Eigen::Vector3d::Zero(); // Feed forward torque
  Eigen::Vector3d qDes = Eigen::Vector3d::Zero();
  Eigen::Vector3d qdDes = Eigen::Vector3d::Zero();
  Eigen::Matrix3d kpJoint = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d kdJoint = Eigen::Matrix3d::Zero();
};

/*!
 * Data returned from the legs to the control code.
 */
struct LegControllerData: public LimbData 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData():LimbData()
  { 
    this->q = Eigen::Vector3d::Zero(); 
    this->qd = Eigen::Vector3d::Zero(); 
    this->tauEstimate = Eigen::Vector3d::Zero(); 
    zero();
  }
  virtual ~LegControllerData(){}

  virtual void zero();
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 */
class LegController 
{
 public:
  LegController() 
  {
    //for (auto& data : datas) data.setQuadruped(_quadruped);
    datas = new LimbData* [4];
    for(int i(0); i<4; ++i) datas[i] = new LegControllerData(); 
  }
  
  ~LegController()
  { 
    for(int i(0); i<4; ++i) delete datas[i]; 
    delete [] datas;
  }

  void zeroCommand();
  void edampCommand(double gain); // RobotType robot, 
  void updateData(const LowState * go2Data);
  void updateCommand(LowCmd * go2Command);

  LegControllerCommand commands[4];
  LimbData** datas;
};

} // namespace go2_system
} // namespace legged_software

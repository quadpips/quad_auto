#pragma once

/*!
 * @file System.hpp
 * @brief System includes
 * 1) Robot dynamic model
 * 2) Controller: MPC, WBC, Backflip, ...
 * 3) State Machine
 * 4) State estimator
 */

#include <stdio.h>

#include <vector>

#include "legged_common/dynamics/FBModelState.h"

// #include <Eigen/StdVector>

namespace legged_software {
namespace legged_common {

class System{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    System(){}
    virtual ~System(){ }
    virtual bool initialization() = 0;
    virtual void onestep_forward() = 0;
    // virtual void renderSystem() = 0 ;
    virtual bool Estop() = 0;
    virtual void updateFBModelStateEstimate() = 0;

    void setFBModelState(const FBModelState & state){
      _state = state;
    }

    bool _resetState = false;
    bool _trainingComplete = false;

    float getOnestepUsec(){
      return _update_dt*1.e6;
    }

    virtual void onestep_forward_test(){}

  protected:
    float _update_dt;
    FBModelState _state;
    // bool _is_sim = true;
};

} // namespace legged_common
} // namespace legged_software

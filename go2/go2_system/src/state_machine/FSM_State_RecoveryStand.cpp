/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "go2_system/state_machine/FSM_State_RecoveryStand.h"

namespace legged_software {
namespace go2_system {
/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSM_State_RecoveryStand::FSM_State_RecoveryStand(ControlFSMData* _controlFSMData)
    : FSM_State(_controlFSMData, FSM_StateName::RECOVERY_STAND, "RECOVERY_STAND")
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  zero_vec3.setZero();
  // goal configuration
  
  // Folding
  fold_jpos[0] << 0.0f, 1.40, -2.70;
  fold_jpos[1] << 0.0f, 1.40, -2.70;
  fold_jpos[2] << 0.0f, 1.40, -2.70;
  fold_jpos[3] << 0.0f, 1.40, -2.70;

  // Stand Up
  stand_jpos[0] << 0.0f, 0.72, -1.44f;
  stand_jpos[1] << 0.0f, 0.72, -1.44f;
  stand_jpos[2] << 0.0f, 0.72, -1.44f;
  stand_jpos[3] << 0.0f, 0.72, -1.44f;
  
  // Rolling
  rolling_jpos[0] << -0.80f, 2.40f, -2.77f;
  rolling_jpos[1] <<  1.30f, 3.30f, -2.77f;
  rolling_jpos[2] << -0.80f, 2.40f, -2.77f;
  rolling_jpos[3] <<  1.30f, 3.30f, -2.77f;

}

void FSM_State_RecoveryStand::onEnter() 
{
    // printf("[FSM_State_RecoveryStand] onEnter()");

    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset iteration counter
    iter = 0;
    _state_iter = 0;
    
    // initial configuration, position
    for(size_t i(0); i < 4; ++i) 
    {
        initial_jpos[i] = this->_data->_legController->datas[i]->q;
    }

    double body_height = this->_data->_stateEstimator->getResult().position[2];

    _flag = FoldLegs;
    if( !_UpsideDown() ) // Proper orientation
    { 
        if (  (0.2 < body_height) && (body_height < 0.45) )
        {
            printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
            _flag = StandUp;
        } else
        {
            printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
        }
    }else
    {
        printf("[Recovery Balance] UpsideDown (%d) \n", _UpsideDown() );
    }
    _motion_start_iter = 0;
}

bool FSM_State_RecoveryStand::_UpsideDown()
{
    //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
    if(this->_data->_stateEstimator->getResult().rBody(2,2) < 0)
    {
        return true;
    }
    return false;
}

//Assigns rollover set points based on roll_left global variable
void FSM_State_RecoveryStand::_AssignRollDirection()
{
    if (roll_left) //set points for rolling left
    {
        rolling_jpos[0] << -0.8f, 2.4f, -2.77f;
        rolling_jpos[1] << 1.3f, 3.3f, -2.77f;
        rolling_jpos[2] << -0.8f, 2.4f, -2.77f;
        rolling_jpos[3] << 1.3f, 3.3f, -2.77f;
    } else //set points for rolling right
    { 
        rolling_jpos[0] << -1.3f, 3.3f, -2.77f;
        rolling_jpos[1] << 0.8f, 2.4f, -2.77f;
        rolling_jpos[2] << -1.3f, 3.3f, -2.77f;
        rolling_jpos[3] << 0.8f, 2.4f, -2.77f;
    }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_RecoveryStand::run() 
{
    switch (_flag)
    {
        case StandUp:
            _StandUp(_state_iter - _motion_start_iter);
            break;
        case FoldLegs:
            _FoldLegs(_state_iter - _motion_start_iter);
            break;
        case RollOver:
            _RollOver(_state_iter - _motion_start_iter);
            break;
    }

 ++_state_iter;
}



void FSM_State_RecoveryStand::_SetJPosInterPts(
    const size_t & curr_iter, 
    const size_t & max_iter, const int & leg, 
    const Eigen::Vector3d & ini, const Eigen::Vector3d & fin)
{
    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if (curr_iter <= max_iter) 
    {
        b = (float)curr_iter/(float)max_iter;
        a = 1.f - b;
    }

    // compute setpoints
    Eigen::Vector3d inter_pos = a * ini + b * fin;

    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3);
}

void FSM_State_RecoveryStand::_RollOver(const int & curr_iter){

  for(size_t i(0); i<4; ++i){
    _SetJPosInterPts(curr_iter, rollover_ramp_iter, i, 
        initial_jpos[i], rolling_jpos[i]);
  }

  if(curr_iter > rollover_ramp_iter + rollover_settle_iter){
    _flag = FoldLegs;
    for(size_t i(0); i<4; ++i) initial_jpos[i] = rolling_jpos[i];
    _motion_start_iter = _state_iter+1;

    roll_left=!roll_left; 
    _AssignRollDirection();
  }
}

void FSM_State_RecoveryStand::_StandUp(const int & curr_iter)
{
    double body_height = this->_data->_stateEstimator->getResult().position[2];
    bool something_wrong(false);

    if ( _UpsideDown() || (body_height < 0.1 ) ) 
    { 
        something_wrong = true;
    }

    if ( (curr_iter > floor(standup_ramp_iter*0.7) ) && something_wrong)
    {
        // If body height is too low because of some reason 
        // even after the stand up motion is almost over 
        // (Can happen when E-Stop is engaged in the middle of Other state)
        for(size_t i(0); i < 4; ++i) 
        {
            initial_jpos[i] = this->_data->_legController->datas[i]->q;
        }
        _flag = FoldLegs;
        _motion_start_iter = _state_iter+1;

        printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n", 
            body_height, _UpsideDown() );

    } else
    {
        for(size_t leg(0); leg<4; ++leg)
        {
            _SetJPosInterPts(curr_iter, standup_ramp_iter, 
                leg, initial_jpos[leg], stand_jpos[leg]);
        }
    }

    Eigen::Vector4d se_contactState(0.5, 0.5, 0.5, 0.5);
    this->_data->_stateEstimator->setContactEstimate(se_contactState);

}

void FSM_State_RecoveryStand::_FoldLegs(const int & curr_iter)
{
    for (size_t i(0); i<4; ++i)
    {
        _SetJPosInterPts(curr_iter, fold_ramp_iter, i, 
            initial_jpos[i], fold_jpos[i]);
    }
    if (curr_iter >= fold_ramp_iter + fold_settle_iter)
    {
        if (_UpsideDown())
        {
            _flag = RollOver;
            for (size_t i(0); i<4; ++i) 
                initial_jpos[i] = fold_jpos[i];
        } else 
        {
            _flag = StandUp;
            for (size_t i(0); i<4; ++i) 
                initial_jpos[i] = fold_jpos[i];
        }
        _motion_start_iter = _state_iter + 1;
    }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_RecoveryStand::checkTransition() 
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->control_mode) 
  {
    case FSM_StateName::RECOVERY_STAND:
      break;

    case FSM_StateName::PASSIVE: 
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case FSM_StateName::PERCEPTIVELOCOMOTIONOCS2:
      this->nextStateName = FSM_StateName::PERCEPTIVELOCOMOTIONOCS2;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << FSM_StateName::RECOVERY_STAND << " to "
                << this->_data->userParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
TransitionData FSM_State_RecoveryStand::transition() 
{
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::PERCEPTIVELOCOMOTIONOCS2:
      this->transitionData.done = true;
      break;    
      
    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
void FSM_State_RecoveryStand::onExit() 
{
  // Nothing to clean up when exiting
}

} // namespace go2_system
} // namespace legged_software

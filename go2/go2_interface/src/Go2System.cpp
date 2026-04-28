/*!
 * @file Go2System.hpp
 * @brief Manages the modules in the Go2 system.
 */

// Go2System
#include "go2_interface/Go2System.hpp"

#include "go2_estimators/observers/OrientationObserver.h"
#include "go2_estimators/estimators/CheaterPositionVelocityEstimator.h"
#include "go2_estimators/estimators/InEKFEstimator.h"

namespace legged_software {
namespace go2_interface {

/*
*   Constructor
*/
Go2System::Go2System(const rclcpp::Node::SharedPtr & node) : System() {
  // Maintain ROS2 node
  node_ = node;

  // Initialize TF buffer and listener
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  RCLCPP_INFO_STREAM(node_->get_logger(), "*** GO2 ***\n");

  // Create leg controller
  _legController = new LegController();
  this->_update_dt = _sys_parameters.controller_dt;
}

/*
*   Create instances of system modules and prepare for operation
*/
bool Go2System::initialization()
{
  // Check if system is already initialized
  if (_initialized)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[GO2 System - Warning] Multiple Initialization Attempts\n");
    return _initialized;
  } 
  else
  {
    // Initialize container for state estimators
    _stateEstimator = new StateEstimatorContainer(
        _sys_cheaterState, 
        _sys_imuData,
        &_legController->datas[0],
        &_sys_parameters
    );
    
    // Initialize finite state machine for control
    _controlFSM = new ControlFSM(
      node_,
      _stateEstimator,
      _legController, 
      _highcmd,
      &_sys_parameters
    );

    // Initialize the InEKF state estimator
    _initializeStateEstimator(false);

    // Initialize the leg controller
    _legController->zeroCommand();

    // Mark system as initialized
    _initialized = true;

    return _initialized;
  }
}

/*
*   Run system for one timestep (retrieve sensor data, get state estimate, get leg commands)
*/
void Go2System::onestep_forward()
{
  // Check if we should enable cheater mode
  if (!_cheaterModeEnabled && _sys_parameters.cheater_mode) 
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[Go2System] Transitioning to Cheater Mode...\n");
    _initializeStateEstimator(true);
    _cheaterModeEnabled = true;
  }
  
  // Check if we should disable cheater mode
  if (_cheaterModeEnabled && !_sys_parameters.cheater_mode) 
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[Go2System] Transitioning from Cheater Mode...\n");
    _initializeStateEstimator(false);
    _cheaterModeEnabled = false;
  }

  // Update joint positions and velocities
  _legController->updateData(_go2State);

  // Clear previous leg controller commands
  _legController->zeroCommand();

  // Run state estimators in container
  _stateEstimator->run();

  // FSM state controller
  _controlFSM->runFSM();
  
  // Update command for robot
  _legController->updateCommand(_go2Command);
}

/*
*   Add desired state estimator to container
*/
void Go2System::_initializeStateEstimator(bool cheaterMode) 
{
  // Clear all state estimators from container
  _stateEstimator->removeAllEstimators();

  // Set initial contact estimate
  Eigen::Vector4d contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactEstimate(contactDefault);

  // Check if cheater mode is enabled
  if (cheaterMode) 
  {
    _stateEstimator->addEstimator<go2_estimators::CheaterOCS2OrientationObserver>();
    _stateEstimator->addEstimator<go2_estimators::CheaterPositionVelocityEstimator>();
  } 

  // Else enable fall-back estimator (InEKF)
  else 
  {
    RCLCPP_WARN(node_->get_logger(), "Using InEKF for state estimation");
    _stateEstimator->addEstimator<go2_estimators::VectorNavOCS2OrientationObserver>();
    _stateEstimator->addEstimator<go2_estimators::InEKFEstimator>(node_);
  }
}

/*
*   Run state estimators but send no commands
*/
bool Go2System::Estop()
{
  // Initialize control FSM
  _controlFSM->initialize();

  // Update joint positions and velocities
  _legController->updateData(_go2State);

  // Run state estimators in container
  _stateEstimator->run();

  // Send zero commands to each leg
  for (int leg = 0; leg < legged_software::go2_system::num_leg; leg++) 
  {
    _legController->commands[leg].zero();
  }
  _legController->updateCommand(_go2Command);

  return true;
}

/*
*   Update local information about pose and joints for visualization
*/
void Go2System::updateFBModelStateEstimate()
{
  // Get updated estimate pose
  _estimated_state.bodyOrientation = _stateEstimator->getResult().orientation;
  _estimated_state.bodyPosition = _stateEstimator->getResult().position;

  // Get update joint positions
  for (int leg = 0; leg < legged_software::go2_system::num_leg; leg++)
  {
    for (int jidx = 0; jidx < 3; ++jidx)
    {
      _estimated_state.q[3*leg + jidx] = _legController->datas[leg]->q[jidx];
    }
  }
}

} // namespace go2_interface
} // namespace legged_software
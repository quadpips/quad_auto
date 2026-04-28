/*!
 * @file Go2System.hpp
 * @brief Manages the modules in the Go2 system.
 */

#pragma once

// Container classes
#include <legged_common/robots/System.hpp>
#include <legged_common/control/ctrl_utils/QuadrupedParameters.h>
#include <legged_common/control/ctrl_utils/HighCmd.h>

// Go2-specific classes
#include <go2_system/state_machine/ControlFSM.h>
#include <go2_system/controllers/auxillary/LegController.h>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"

// Unitree
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace legged_software {
namespace go2_interface {

using legged_common::System;
using legged_common::QuadrupedParameters;
using legged_common::FBModelState;
using legged_common::HighCmdCustom;

using go2_system::LegController;
using go2_system::ControlFSM;

using go2_estimators::ImuData;
using go2_estimators::CheaterState;
using go2_estimators::StateEstimatorContainer;

class Go2System: public System {

  public:
    // Constructor
    Go2System(const rclcpp::Node::SharedPtr & node);
    virtual ~Go2System()
    {
      delete _controlFSM;
      delete _legController;
      delete _stateEstimator;
    }

    // Class methods
    const QuadrupedParameters & getParameters(){
      return _sys_parameters; 
    }

    // Virtual functions
    virtual bool initialization();
    virtual bool Estop();
    virtual void updateFBModelStateEstimate();
    virtual void onestep_forward();

    // Pointers for accessing ROS2 data
    unitree_go::msg::LowCmd * _go2Command;
    unitree_go::msg::LowState * _go2State;

    // Variables for maintaining system state
    ImuData* _sys_imuData;
    CheaterState* _sys_cheaterState;
    HighCmdCustom* _highcmd;
    FBModelState _estimated_state;
    QuadrupedParameters _sys_parameters;
    StateEstimatorContainer* _stateEstimator;
    bool _cheaterModeEnabled = false;

  protected:
    // System initialization
    bool _initialized = false;
    void _initializeStateEstimator(bool cheaterMode = false);

    // Pointers to instances of modules
    ControlFSM* _controlFSM;
    LegController* _legController;

    // ROS2
    rclcpp::Node::SharedPtr node_;  
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
};

} // namespace go2_interface
} // namespace legged_software

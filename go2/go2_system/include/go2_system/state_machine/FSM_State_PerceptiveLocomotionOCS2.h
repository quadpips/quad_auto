#pragma once

// other modules
#include <angles/angles.h>

// legged modules
#include <go2_wbc/PerceptiveLocomotionOCS2WBC/WbcBase.hpp>
#include <go2_wbc/PerceptiveLocomotionOCS2WBC/Factory.hpp>

// go2 modules
#include <go2_system/state_machine/FSM_State.h>

// ocs2 modules
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>
// #include <ocs2_custom_quadruped_interface/QuadrupedMpc.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
// #include <ocs2_switched_model_interface/terrain/TerrainPlane.h>
// #include <ocs2_custom_quadruped_interface/LocalTerrainReceiver.h>
#include <ocs2_custom_quadruped_interface/CustomSwingPlanningVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainPlaneVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainReceiver.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedVisualizer.h>

#include "rclcpp/rclcpp.hpp"

namespace legged_software {
namespace go2_system {

using namespace ocs2;
using namespace switched_model;

/**
 *
 */
class FSM_State_PerceptiveLocomotionOCS2 : public FSM_State 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FSM_State_PerceptiveLocomotionOCS2(const rclcpp::Node::SharedPtr & node, ControlFSMData* _controlFSMData);
        ~FSM_State_PerceptiveLocomotionOCS2();

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

    private:
        // void runOCS2Thread();
        
        SystemObservation generateSystemObservation();

        void setupLeggedRobotInterface(const std::string& taskFile, 
                                        const std::string& urdfFile,
                                        const std::string& frameFile); // const std::string& envFile
        void setupMpc(const std::string& taskFile, 
                        const std::string& sqpFile);
        void setupMrt(const std::string& sqpFile);

        void printObservation(const SystemObservation & observation);
        void printState(const vector_t & state);
        void printInput(const vector_t & input);

        vector_t getRbdState();
        vector_t extractCentroidalState(const vector_t & rbdState);

        bool stateSafetyCheck(const vector_t & state);
        bool inputSafetyCheck(const vector_t & input);
        bool torqueSafetyCheck(const std::vector<JointCommand> & commands,
                                const vector_t & state,
                                const vector_t & input);

        void sendZeroCommand();

        std::shared_ptr<CustomQuadrupedInterface> legged_interface_;

        std::shared_ptr<WbcBase> wbcPtr_;

        std::shared_ptr<MPC_BASE> mpcPtr_;

        std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;

        std::shared_ptr<CustomQuadrupedVisualizer> visualizer_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;
        // SystemObservation current_observation_;

        std::thread ocs2_thread_;
        std::atomic_bool controller_running_, mpc_running_{};
        // size_t planned_mode_= switched_model::ModeNumber::STANCE;  // The mode that is active at the time the policy is evaluated at.

        // Create the joint P gain matrix
        Eigen::Matrix3d kpMat_joint;

        // Create the joint D gain matrix
        Eigen::Matrix3d kdMat_joint;

        // Keep track of the control iterations
        int iter = 0;

        rclcpp::Node::SharedPtr node_;

        const std::string robotName = "go2";
        scalar_t timeSinceLastLog_ = 1e5;
        // scalar_t initTime_ = 0.0;
        size_t desiredMode = switched_model::ModeNumber::STANCE; // Default mode
        vector_t desiredInput = vector_t::Zero(switched_model::INPUT_DIM);
        // scalar_t tNow_ = 0.0;

        SystemObservation pastObservation_;

        bool perceptiveOCS2Safe = true;

        vector3_t maxTorqueObserved_ = vector3_t::Zero();
};


} // namespace go2_system
} // namespace legged_software

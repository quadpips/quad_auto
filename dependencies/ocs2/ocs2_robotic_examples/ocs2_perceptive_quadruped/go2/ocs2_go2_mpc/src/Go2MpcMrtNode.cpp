//
// Created by rgrandia on 31.03.22.
//

#include "rclcpp/rclcpp.hpp"

#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_go2_mpc/Go2Interface.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedMpc.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_sqp/SqpMpc.h>

#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_custom_quadruped_interface/CustomSwingPlanningVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainPlaneVisualizer.h>
#include <ocs2_custom_quadruped_interface/CustomTerrainReceiver.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedVisualizer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"

using namespace ocs2;
using namespace switched_model;
using MN = switched_model::ModeNumber;

class MpcMrtDebugger
{
    public:
        MpcMrtDebugger(const rclcpp::Node::SharedPtr& node)
        {
            const std::string robotName = "go2";
            node_ = node;

            RCLCPP_INFO_STREAM(node_->get_logger(), "Starting MpcMrtDebugger for robot: " << robotName);

            const std::string taskFile = node_->get_parameter("taskFile").as_string();
            const std::string urdfFile = node_->get_parameter("urdfFile").as_string();
            const std::string frameFile = node_->get_parameter("frameFile").as_string();
            const std::string sqpFile = node_->get_parameter("sqpFile").as_string();    
            
            std::string urdfString = go2::getUrdfString(urdfFile);

            legged_interface_ = go2::getGo2Interface(urdfString, taskFile, frameFile);

            // ====== Create MPC solver ======== //
            ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile);
            ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile);
            mpcPtr_ = std::make_shared<ocs2::SqpMpc>(mpcSettings, 
                                                        sqpSettings,
                                                        legged_interface_->getOptimalControlProblem(),
                                                        legged_interface_->getInitializer());

            // ======= Placeholder for synchronized modules, not using yet ======== //

            auto solverModules = legged_interface_->getSynchronizedModules();

            // Gait Receiver
            auto gaitReceiver =
                std::make_shared<GaitReceiver>(node_, 
                                                legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), 
                                                robotName);
            solverModules.push_back(gaitReceiver);

            // Terrain Receiver
            auto terrainReceiver = std::make_shared<CustomTerrainReceiverSynchronizedModule>(
                legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), node_);
            solverModules.push_back(terrainReceiver);

            // Terrain plane visualization
            auto terrainVisualizer = std::make_shared<CustomTerrainPlaneVisualizerSynchronizedModule>(
                legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node_);
            solverModules.push_back(terrainVisualizer);

            // Swing planner
            auto swingPlanningVisualizer = std::make_shared<CustomSwingPlanningVisualizer>(
                legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node_);
            solverModules.push_back(swingPlanningVisualizer);

            // ROS ReferenceManager; The ReferenceManager will subscribe to "topicPrefix_mode_schedule" and "topicPrefix_mpc_target"
            auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, 
                                                                                        legged_interface_->getReferenceManagerPtr());
            observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);

            rosReferenceManagerPtr->subscribe(node_);
            mpcPtr_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

            // MPC
            mpcPtr_->getSolverPtr()->setSynchronizedModules(solverModules);

            mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpcPtr_);
            mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

            // ====== Set the scenario to the correct interfaces ========
            auto referenceManager = legged_interface_->getSwitchedModelModeScheduleManagerPtr();

            // Create observation
            current_observation_.time = 0.0;
            current_observation_.mode = MN::STANCE;
            current_observation_.state = legged_interface_->getInitialState();
            current_observation_.input.setZero(24);

            ocs2::TargetTrajectories targetTrajectories({ current_observation_.time }, 
                                                        { current_observation_.state },
                                                        { current_observation_.input });

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->reset();
            mpc_mrt_interface_->resetMpcNode(targetTrajectories);
                
            mpc_mrt_interface_->setCurrentObservation(current_observation_);

            RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for initial policy...");
            while (!mpc_mrt_interface_->initialPolicyReceived() && rclcpp::ok()) 
            {
                mpc_mrt_interface_->advanceMpc();
            }
            RCLCPP_INFO_STREAM(node_->get_logger(), "Initial policy received.");

            // Visualization
            visualizer_ = std::make_shared<CustomQuadrupedVisualizer>(legged_interface_->getKinematicModel(), 
                                                                      legged_interface_->getJointNames(), 
                                                                      legged_interface_->getBaseName(),
                                                                      node_,
                                                                      1000);

            // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
            // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        void run()
        {
            try
            {
                // set current observation
                current_observation_.time += 0.02;

                // Where should I pull state from?


                RCLCPP_INFO_STREAM(node_->get_logger(), "observation (pre-mpc): \n");
                RCLCPP_INFO_STREAM(node_->get_logger(), "   time:                " << current_observation_.time);
                RCLCPP_INFO_STREAM(node_->get_logger(), "   mode:                " << current_observation_.mode);
                RCLCPP_INFO_STREAM(node_->get_logger(), "   state: ");
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso orientation: " << current_observation_.state.segment(0,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso position:    " << current_observation_.state.segment(3,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso angular vel: " << current_observation_.state.segment(6,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso linear vel:  " << current_observation_.state.segment(9,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LF joint pos:      " << current_observation_.state.segment(12,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RF joint pos:      " << current_observation_.state.segment(15,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LH joint pos:      " << current_observation_.state.segment(18,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RH joint pos:      " << current_observation_.state.segment(21,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "   input: ");
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LF contact force:  " << current_observation_.input.segment(0,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RF contact force:  " << current_observation_.input.segment(3,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LH contact force:  " << current_observation_.input.segment(6,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RH contact force:  " << current_observation_.input.segment(9,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LF joint vel:      " << current_observation_.input.segment(12,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RF joint vel:      " << current_observation_.input.segment(15,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LH joint vel:      " << current_observation_.input.segment(18,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RH joint vel:      " << current_observation_.input.segment(21,3).transpose());

                // run MPC at current observation
                mpc_mrt_interface_->setCurrentObservation(current_observation_);
                mpc_mrt_interface_->advanceMpc();
                mpc_mrt_interface_->updatePolicy();

                // Evaluate the optimized solution - change to optimal controller
                vector_t desiredState;
                vector_t desiredInput;
                size_t desiredMode;
                mpc_mrt_interface_->evaluatePolicy(current_observation_.time, 
                                                    current_observation_.state, 
                                                    desiredState,
                                                    desiredInput, 
                                                    desiredMode);

                // Set the current observation to the desired state and input

                // WHERE IS STATE SET
                current_observation_.state = desiredState;
                current_observation_.input = desiredInput;
                current_observation_.mode = desiredMode;
                
                RCLCPP_INFO_STREAM(node_->get_logger(), "observation (post-mpc): \n");
                RCLCPP_INFO_STREAM(node_->get_logger(), "   time:                " << current_observation_.time);
                RCLCPP_INFO_STREAM(node_->get_logger(), "   mode:                " << current_observation_.mode);
                RCLCPP_INFO_STREAM(node_->get_logger(), "   state: ");
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso orientation: " << current_observation_.state.segment(0,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso position:    " << current_observation_.state.segment(3,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso angular vel: " << current_observation_.state.segment(6,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     torso linear vel:  " << current_observation_.state.segment(9,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LF joint pos:      " << current_observation_.state.segment(12,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RF joint pos:      " << current_observation_.state.segment(15,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LH joint pos:      " << current_observation_.state.segment(18,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RH joint pos:      " << current_observation_.state.segment(21,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "   input: ");
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LF contact force:  " << current_observation_.input.segment(0,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RF contact force:  " << current_observation_.input.segment(3,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LH contact force:  " << current_observation_.input.segment(6,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RH contact force:  " << current_observation_.input.segment(9,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LF joint vel:      " << current_observation_.input.segment(12,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RF joint vel:      " << current_observation_.input.segment(15,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     LH joint vel:      " << current_observation_.input.segment(18,3).transpose());
                RCLCPP_INFO_STREAM(node_->get_logger(), "     RH joint vel:      " << current_observation_.input.segment(21,3).transpose());

                visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());


                // Publish the observation. Only needed for the command interface
                observation_publisher_->publish(ros_msg_conversions::createObservationMsg(current_observation_));


                // SystemObservation current_observation_;
                // current_observation_ = observation;

                // Publish the transform from world to base
                // geometry_msgs::msg::TransformStamped transformStamped;
                // transformStamped.header.stamp = node_->get_clock()->now();
                // transformStamped.header.frame_id = "world";
                // transformStamped.child_frame_id = "base";
                // transformStamped.transform.translation.x = observation.state(3);
                // transformStamped.transform.translation.y = observation.state(4);
                // transformStamped.transform.translation.z = observation.state(5);
                // tf2::Quaternion q;
                // q.setRPY(observation.state(0), observation.state(1), observation.state(2));
                // transformStamped.transform.rotation.x = q.x();
                // transformStamped.transform.rotation.y = q.y();
                // transformStamped.transform.rotation.z = q.z();
                // transformStamped.transform.rotation.w = q.w();
                // tf_broadcaster_->sendTransform(transformStamped);

            } catch (std::exception& e) 
            {
                std::cout << "MPC failed\n";
                std::cout << e.what() << "\n";
                return;
            }
        }

    private:
        std::shared_ptr<CustomQuadrupedInterface> legged_interface_;
        std::shared_ptr<MPC_BASE> mpcPtr_;
        std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        SystemObservation current_observation_;
        std::shared_ptr<CustomQuadrupedVisualizer> visualizer_;
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;

        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
                                                            "mpc_mrt_debugging_node",
                                                            rclcpp::NodeOptions()
                                                                .allow_undeclared_parameters(true)
                                                                .automatically_declare_parameters_from_overrides(true));

    MpcMrtDebugger mpcMrtDebugger(node);

    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok()) 
    {
        mpcMrtDebugger.run();
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
};
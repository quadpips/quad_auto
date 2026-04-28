/*!
 * @file Go2Bridge.hpp
 * @brief Base class for setting up data bridges between ROS2 data and robot code.
 */

#pragma once

// Go2 interface
#include <go2_interface/Go2System.hpp>
#include <go2_interface_msgs/srv/cheater_mode.hpp>
#include <go2_interface_msgs/srv/go2_cmd.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

// Unitree
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "go2_interface/unitree_ros2/common/motor_crc.h"

// Type aliases
using GO2CmdRequest = go2_interface_msgs::srv::GO2Cmd::Request;
using GO2CmdResponse = go2_interface_msgs::srv::GO2Cmd::Response;
using CheaterModeRequest = go2_interface_msgs::srv::CheaterMode::Request;
using CheaterModeResponse = go2_interface_msgs::srv::CheaterMode::Response;

namespace legged_software {
namespace go2_interface {

class Go2Bridge
{
    public:
        /*
        *   Constructor
        */
        Go2Bridge(const rclcpp::Node::SharedPtr &node, Go2System* system)
        {
            _node = node;
            _system = system;

            // Services
            service_control = _node->create_service<go2_interface_msgs::srv::GO2Cmd>(
                "/ControlMode", 
                [this](
                    const std::shared_ptr<go2_interface_msgs::srv::GO2Cmd::Request> request, 
                    const std::shared_ptr<go2_interface_msgs::srv::GO2Cmd::Response> response
                ) {
                    return ControlModeCallback(request, response);
                }
            );
            service_cheater = node->create_service<go2_interface_msgs::srv::CheaterMode>(
                "/CheaterMode", 
                [this](
                    const std::shared_ptr<go2_interface_msgs::srv::CheaterMode::Request> request,
                    std::shared_ptr<go2_interface_msgs::srv::CheaterMode::Response> response
                ){
                    return CheaterModeCallback(request, response);
                }
            );           

            // For broadcasting frame transforms
            broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(_node);

            // Publishers
            pub_joint_state = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);
        }
        
        /*
        *   Setup and run the system
        */
        void run() 
        {
            RCLCPP_INFO_STREAM(_node->get_logger(), "[Go2Bridge::run()]");

            // Setup pointers for data flow from ROS2 to Go2System
            _system->_go2State = &low_state;
            _system->_go2Command = &low_cmd;
            _system->_sys_imuData = &_imuData;
            _system->_highcmd = &_highlevelCmd;
            _system->_sys_cheaterState = &_cheaterState;

            // Initialize Go2System
            _system->initialization();

            // Start system threads
            _ctrlThread = std::thread(&Go2Bridge::runSystem, this);
            _sdkThread = std::thread(&Go2Bridge::lowCmdWrite, this);
            _visThread = std::thread(&Go2Bridge::systemRendering, this);

            RCLCPP_INFO_STREAM(_node->get_logger(), "[Go2Bridge::run()] finished");
        }    

        /*
        *   Manage system time and move Go2System one step forward
        */
        void runSystem()
        {
            // Get time
            auto now = _node->get_clock()->now();
            auto dt = rclcpp::Duration::from_seconds(0);
            auto dt_loop = rclcpp::Duration::from_seconds(0);
            auto prev = _node->get_clock()->now();

            // Set desired frequency
            float des_dt = 1.0 / 500.0; // 500Hz

            while (rclcpp::ok())
            {
                // Get updated time
                now = _node->get_clock()->now();
                dt = now - prev;
                prev = _node->get_clock()->now();

                // Set time difference in system
                _system->_sys_parameters.controller_dt = dt.seconds();
                _system->_sys_parameters.estimator_dt = dt.seconds();

                // Run Go2System for one step
                if (_iter_run_system < 100)
                {
                    // Estop for the first 50 iterations; 0.1s for 500Hz case
                    _system->Estop();
                } else
                {
                    _system->onestep_forward();
                }
                ++_iter_run_system;

                // Sleep to achieve desired run frequency
                dt_loop = _node->get_clock()->now() - now;
                if (dt_loop.seconds() < des_dt)
                {
                    const rclcpp::Duration duration = rclcpp::Duration::from_seconds(des_dt - dt_loop.seconds());
                    rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
                }
            }
        }         
        
        /*
        *   Update system state for visualization
        */
        void systemRendering()
        {
            // Get time
            rclcpp::Time now = _node->get_clock()->now();
            rclcpp::Duration dt_loop = rclcpp::Duration::from_seconds(0);
            float des_dt = 1.0 / 500.0; // 500Hz
            
            while (rclcpp::ok())
            {
                // Get updated time
                now = _node->get_clock()->now();

                // Update the state estimation
                _system->updateFBModelStateEstimate();

                // Update system with IMU readings
                _imuData.accelerometer[0] = low_state.imu_state.accelerometer[0];
                _imuData.accelerometer[1] = low_state.imu_state.accelerometer[1];
                _imuData.accelerometer[2] = low_state.imu_state.accelerometer[2];

                _imuData.quat = Eigen::Quaterniond(low_state.imu_state.quaternion[0], 
                                                    low_state.imu_state.quaternion[1], 
                                                    low_state.imu_state.quaternion[2], 
                                                    low_state.imu_state.quaternion[3]);
                
                _imuData.gyro[0] = low_state.imu_state.gyroscope[0];
                _imuData.gyro[1] = low_state.imu_state.gyroscope[1];
                _imuData.gyro[2] = low_state.imu_state.gyroscope[2];

                // Publish joint states for visualization in RViz
                publishJointState(low_state, now);

                // Publish estimated frame transformations
                publishEstimateTF(now);

                // Sleep to achieve desired run frequency
                dt_loop = _node->get_clock()->now() - now;
                if (dt_loop.seconds() < des_dt)
                {
                    const rclcpp::Duration duration = rclcpp::Duration::from_seconds(des_dt - dt_loop.seconds());
                    rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
                }
            }
        } 
        
        // Pure virtual functions
        virtual void lowCmdWrite() = 0;

    protected:
        /*
        *   Service callback for activating cheater mode
        */
        void CheaterModeCallback(const std::shared_ptr<go2_interface_msgs::srv::CheaterMode::Request> req,
                                       std::shared_ptr<go2_interface_msgs::srv::CheaterMode::Response> res) 
        {
            // Set system to use cheater mode
            _system->_sys_parameters.cheater_mode = req->cmd;

            // Update response
            res->result = 0; // 0: success, -1: failure
            res->description = "Cheater mode changed to " + std::to_string(req->cmd);

            // Logging
            RCLCPP_INFO(_node->get_logger(), "Cheater mode switch to: [%hd]", req->cmd);

            return;
        }

        /*
        *   Service callback for changing control mode
        */
        void ControlModeCallback(const std::shared_ptr<go2_interface_msgs::srv::GO2Cmd::Request> req,
                                 std::shared_ptr<go2_interface_msgs::srv::GO2Cmd::Response> res)
        {
            // Set system to use desired control mode
            _highlevelCmd.mode = req->cmd;

            // Update response
            res->result = 0;
            res->description = "Get new control mode";

            // Logging
            RCLCPP_INFO(_node->get_logger(), "Control mode switch to: [%hd]", req->cmd);

            return;
        }

        /*
        *   Initialize the low command message
        */
        void InitLowCmd() 
        {
            low_cmd.head[0] = 0xFE;
            low_cmd.head[1] = 0xEF;
            low_cmd.level_flag = 0xFF;
            low_cmd.gpio = 0;

            for (int i = 0; i < 20; i++) 
            {
                low_cmd.motor_cmd[i].mode = (0x01);  // motor switch to servo (PMSM) mode
                low_cmd.motor_cmd[i].q = (PosStopF);
                low_cmd.motor_cmd[i].kp = (0);
                low_cmd.motor_cmd[i].dq = (VelStopF);
                low_cmd.motor_cmd[i].kd = (0);
                low_cmd.motor_cmd[i].tau = (0);
            }
        }

        /*
        *   Verify that the transformation message is complete
        */
        bool healthCheckTransform(const geometry_msgs::msg::TransformStamped & estimate_trans)
        {
            // Check if frame transformation broadcaster is initialized
            if (broadcaster == nullptr)
            {
                RCLCPP_ERROR_STREAM(_node->get_logger(), "Broadcaster is not initialized!");
                return false;
            }

            // Check if transformation message has correct metadata
            if (estimate_trans.header.frame_id.empty() || estimate_trans.child_frame_id.empty())
            {
                RCLCPP_ERROR_STREAM(_node->get_logger(), "Transform is not properly initialized!");
                return false;
            }

            // Check position and orientation for NaN values
            if (std::isnan(estimate_trans.transform.translation.x) ||
                std::isnan(estimate_trans.transform.translation.y) ||
                std::isnan(estimate_trans.transform.translation.z) ||
                std::isnan(estimate_trans.transform.rotation.w) ||
                std::isnan(estimate_trans.transform.rotation.x) ||
                std::isnan(estimate_trans.transform.rotation.y) ||
                std::isnan(estimate_trans.transform.rotation.z))
            {
                RCLCPP_ERROR_STREAM(_node->get_logger(), "Transform contains NaN values!");
                return false;
            }

            // Check if the quaternion is normalized
            double norm = std::sqrt(std::pow(estimate_trans.transform.rotation.w, 2) +
                                    std::pow(estimate_trans.transform.rotation.x, 2) +
                                    std::pow(estimate_trans.transform.rotation.y, 2) +
                                    std::pow(estimate_trans.transform.rotation.z, 2));
            if (std::abs(norm - 1.0) > 1e-6)
            {
                RCLCPP_ERROR_STREAM(_node->get_logger(), 
                    "Quaternion is not normalized. W: " << estimate_trans.transform.rotation.w
                    << ", X: " << estimate_trans.transform.rotation.x
                    << ", Y: " << estimate_trans.transform.rotation.y
                    << ", Z: " << estimate_trans.transform.rotation.z);

                return false;
            }

            return true;
        }        

        /*
        *   Publish frame transformation estimate from odom frame to base and base_aligned
        */
        void publishEstimateTF(const rclcpp::Time &timestamp)
        {
            // Initialize messages
            geometry_msgs::msg::TransformStamped estimate_odom_to_base_aligned_trans;
            geometry_msgs::msg::TransformStamped estimate_odom_to_base_trans;

            // Construct message
            estimate_odom_to_base_trans.header.stamp = timestamp;
            estimate_odom_to_base_trans.header.frame_id = "odom";
            estimate_odom_to_base_trans.child_frame_id = "base";        
            estimate_odom_to_base_trans.transform.translation.x = _system->_estimated_state.bodyPosition[0];
            estimate_odom_to_base_trans.transform.translation.y = _system->_estimated_state.bodyPosition[1];
            estimate_odom_to_base_trans.transform.translation.z = _system->_estimated_state.bodyPosition[2];
            estimate_odom_to_base_trans.transform.rotation.w = _system->_estimated_state.bodyOrientation.w();
            estimate_odom_to_base_trans.transform.rotation.x = _system->_estimated_state.bodyOrientation.x();
            estimate_odom_to_base_trans.transform.rotation.y = _system->_estimated_state.bodyOrientation.y();
            estimate_odom_to_base_trans.transform.rotation.z = _system->_estimated_state.bodyOrientation.z();

            // Verify message is healthy
            if (healthCheckTransform(estimate_odom_to_base_trans))
            {
                // Publish transform
                broadcaster->sendTransform(estimate_odom_to_base_trans);

                // Construct aligned transformation (only includes yaw, no pitch or roll)
                estimate_odom_to_base_aligned_trans = estimate_odom_to_base_trans;
                estimate_odom_to_base_aligned_trans.child_frame_id = "base_aligned";
                
                // Initialize rotation
                tf2::Quaternion q(estimate_odom_to_base_aligned_trans.transform.rotation.x,
                                    estimate_odom_to_base_aligned_trans.transform.rotation.y,
                                    estimate_odom_to_base_aligned_trans.transform.rotation.z,
                                    estimate_odom_to_base_aligned_trans.transform.rotation.w);
                tf2::Matrix3x3 mat(q);

                // Extract RPY and only keep yaw
                tf2Scalar roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);
                q.setRPY(0.0, 0.0, yaw); // yaw

                // Construct message
                estimate_odom_to_base_aligned_trans.transform.rotation.x = q.x();
                estimate_odom_to_base_aligned_trans.transform.rotation.y = q.y();
                estimate_odom_to_base_aligned_trans.transform.rotation.z = q.z();
                estimate_odom_to_base_aligned_trans.transform.rotation.w = q.w();

                // Publish transform
                broadcaster->sendTransform(estimate_odom_to_base_aligned_trans);
            }
            else
            {
                RCLCPP_ERROR_STREAM(_node->get_logger(), "[systemRendering()] Transform is not healthy, skipping broadcast");
            }            
        }

        /*
        *   Publish joint state readings
        */
        void publishJointState(const unitree_go::msg::LowState &low_state, const rclcpp::Time &timestamp) const 
        {
            // Initialize message
            sensor_msgs::msg::JointState joint_state_msg;

            // Assign the list of joint names
            joint_state_msg.name = _joint_names;

            // Resize the vectors so we can index into them.
            joint_state_msg.position.resize(12);
            joint_state_msg.velocity.resize(12);
            joint_state_msg.effort.resize(12);

            joint_state_msg.header.stamp = timestamp;

            // Assign the joint angle, velocity and torque.
            for (int i = 0; i < 12; i++) 
            {
                joint_state_msg.position[i] = low_state.motor_state[i].q;
                joint_state_msg.velocity[i] = low_state.motor_state[i].dq;
                joint_state_msg.effort[i] = low_state.motor_state[i].tau_est;
            }

            // Publish message
            pub_joint_state->publish(joint_state_msg);
        }

        // Initialize Go2System
        Go2System* _system = nullptr;

        // Maintain how many iterations the system has run
        unsigned long long _iter_run_system = 0;

        // ROS2 services
        rclcpp::Service<go2_interface_msgs::srv::GO2Cmd>::SharedPtr service_control;
        rclcpp::Service<go2_interface_msgs::srv::CheaterMode>::SharedPtr service_cheater;

        // Maintain ground-truth simulation state
        CheaterState _cheaterState;

        // ROS2 helpers
        rclcpp::Node::SharedPtr _node;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;

        // ROS2 messages for maintaining data
        unitree_go::msg::LowCmd low_cmd;
        unitree_go::msg::LowState low_state;

        // Threads for running the system
        std::thread _sdkThread;
        std::thread _ctrlThread;
        std::thread _visThread;

        // Maintain Go2System data
        ImuData _imuData;
        HighCmdCustom _highlevelCmd;

        // Go2 parameters
        std::vector<std::string> _feet_names = { "FL_foot", "FR_foot", "RL_foot", "RR_foot" };
        std::vector<std::string> _joint_names = { "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                                                    "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  
                                                    "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint",
                                                    "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint"};

};

} // namespace go2_interface
} // namespace legged_software
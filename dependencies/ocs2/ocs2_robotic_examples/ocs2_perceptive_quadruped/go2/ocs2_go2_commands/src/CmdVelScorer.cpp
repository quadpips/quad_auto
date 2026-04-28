//
// Created by qiayuan on 2022/7/24.
//

#include "ocs2_go2_commands/Go2VelCommandNode.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <angles/angles.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

using namespace ocs2;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using namespace std::chrono_literals;

class CmdVelScorer : public rclcpp::Node
{
    public:
        CmdVelScorer() : Node("cmd_vel_scorer")
        {
            // RCLCPP_INFO(this->get_logger(), "CmdVelScorer Node has been started.");
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&CmdVelScorer::cmdVelCallback, this, std::placeholders::_1));

            mpc_observation_sub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
                "/go2_mpc_observation", 10, std::bind(&CmdVelScorer::mpcObservationCallback, this, std::placeholders::_1));

            latest_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/latest_cmd_vel", 10);
            torso_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/go2_torso_pose", 10);
            torso_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/go2_torso_velocity", 10);
            joint_pos_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/go2_joint_states", 10);

            FL_contact_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go2_FL_contact_force", 10);
            FR_contact_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go2_FR_contact_force", 10);
            RL_contact_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go2_RL_contact_force", 10);
            RR_contact_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go2_RR_contact_force", 10);

            // Call reportCmdVelTrackingError function every second
            timer_ = this->create_wall_timer(0.5s, std::bind(&CmdVelScorer::reportCmdVelTrackingError, this));
        }

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);
            latest_cmd_vel_.twist = *msg;
            // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: [%f, %f, %f]",
            //             latest_cmd_vel_.twist.linear.x, latest_cmd_vel_.twist.linear.y, latest_cmd_vel_.twist.linear.z);
        }

        void mpcObservationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(latest_observation_mutex_);
            latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);

            // scalar_t time = latest_observation_.time;

            // remove old observations
            // scalar_t time_window = 3.0; // seconds, around three gait cycles
            // for (auto it = observations_.begin(); it != observations_.end();)
            // {
            //     if (it->time < time - time_window)
            //     {
            //         observations_.erase(it, observations_.end());
                    
            //         // always pushing to back, so can erase once we find first old observation
            //         break;
            //     }
            //     else
            //     {
            //         ++it;
            //     }
            // }

            // add new observation
            observations_.push_back(latest_observation_);

            rclcpp::Time now = this->now();

            // publish latest cmd_vel
            geometry_msgs::msg::TwistStamped latest_cmd_vel_msg;
            latest_cmd_vel_msg.header.stamp = now;
            latest_cmd_vel_msg.header.frame_id = "base";
            latest_cmd_vel_msg.twist = latest_cmd_vel_.twist;
            latest_cmd_vel_pub_->publish(latest_cmd_vel_msg);

            // publish torso pose
            geometry_msgs::msg::PoseStamped torso_pose;
            torso_pose.header.stamp = now;
            torso_pose.header.frame_id = "world";
            torso_pose.pose.position.x = latest_observation_.state(3);
            torso_pose.pose.position.y = latest_observation_.state(4);
            torso_pose.pose.position.z = latest_observation_.state(5);
            Eigen::Quaternion<scalar_t> orientation = ocs2::getQuaternionFromEulerAnglesXyz(vector3_t(latest_observation_.state.head(3)));
            torso_pose.pose.orientation.x = orientation.x();
            torso_pose.pose.orientation.y = orientation.y();
            torso_pose.pose.orientation.z = orientation.z();
            torso_pose.pose.orientation.w = orientation.w();
            torso_pose_pub_->publish(torso_pose);

            // publish joint positions and velocities
            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = now;
            joint_state.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
            joint_state.position = {
                latest_observation_.state(12), latest_observation_.state(13), latest_observation_.state(14),
                latest_observation_.state(15), latest_observation_.state(16), latest_observation_.state(17),
                latest_observation_.state(18), latest_observation_.state(19), latest_observation_.state(20),
                latest_observation_.state(21), latest_observation_.state(22), latest_observation_.state(23) };
            joint_state.velocity = {
                latest_observation_.input(12), latest_observation_.input(13), latest_observation_.input(14),
                latest_observation_.input(15), latest_observation_.input(16), latest_observation_.input(17),
                latest_observation_.input(18), latest_observation_.input(19), latest_observation_.input(20),
                latest_observation_.input(21), latest_observation_.input(22), latest_observation_.input(23) };
            joint_pos_pub_->publish(joint_state);

            // publish torso velocity
            geometry_msgs::msg::TwistStamped torso_velocity;
            torso_velocity.header.stamp = now;
            torso_velocity.header.frame_id = "base";
            torso_velocity.twist.linear.x = latest_observation_.state(9);
            torso_velocity.twist.linear.y = latest_observation_.state(10);
            torso_velocity.twist.linear.z = latest_observation_.state(11);
            torso_velocity.twist.angular.x = latest_observation_.state(6);
            torso_velocity.twist.angular.y = latest_observation_.state(7);
            torso_velocity.twist.angular.z = latest_observation_.state(8);
            torso_vel_pub_->publish(torso_velocity);

            // publish contact forces
            geometry_msgs::msg::WrenchStamped FL_contact_force;
            FL_contact_force.header.stamp = now;
            FL_contact_force.header.frame_id = "odom";
            FL_contact_force.wrench.force.x = latest_observation_.input(0);
            FL_contact_force.wrench.force.y = latest_observation_.input(1);
            FL_contact_force.wrench.force.z = latest_observation_.input(2);
            geometry_msgs::msg::WrenchStamped FR_contact_force;
            FR_contact_force.header.stamp = now;
            FR_contact_force.header.frame_id = "odom";
            FR_contact_force.wrench.force.x = latest_observation_.input(3);
            FR_contact_force.wrench.force.y = latest_observation_.input(4);
            FR_contact_force.wrench.force.z = latest_observation_.input(5);
            geometry_msgs::msg::WrenchStamped RL_contact_force;
            RL_contact_force.header.stamp = now;
            RL_contact_force.header.frame_id = "odom";
            RL_contact_force.wrench.force.x = latest_observation_.input(6);
            RL_contact_force.wrench.force.y = latest_observation_.input(7);
            RL_contact_force.wrench.force.z = latest_observation_.input(8);
            geometry_msgs::msg::WrenchStamped RR_contact_force;
            RR_contact_force.header.stamp = now;
            RR_contact_force.header.frame_id = "odom";
            RR_contact_force.wrench.force.x = latest_observation_.input(9);
            RR_contact_force.wrench.force.y = latest_observation_.input(10);
            RR_contact_force.wrench.force.z = latest_observation_.input(11);

            FL_contact_force_pub_->publish(FL_contact_force);
            FR_contact_force_pub_->publish(FR_contact_force);
            RL_contact_force_pub_->publish(RL_contact_force);
            RR_contact_force_pub_->publish(RR_contact_force);

            // RCLCPP_INFO(this->get_logger(), "Received mpc_observation at time: %f", time);
            // RCLCPP_INFO(this->get_logger(), "State: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
            //             latest_observation_.state(0), latest_observation_.state(1), latest_observation_.state(2),
            //             latest_observation_.state(3), latest_observation_.state(4), latest_observation_.state(5),
            //             latest_observation_.state(6), latest_observation_.state(7), latest_observation_.state(8),
            //             latest_observation_.state(9), latest_observation_.state(10), latest_observation_.state(11));
        }

        void reportCmdVelTrackingError()
        {
            std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);
            std::lock_guard<std::mutex> lock_obs(latest_observation_mutex_);

            // calculate running average of observations
            if (observations_.empty())
                return;

            vector3_t avg_lin_vel = vector3_t::Zero();
            vector3_t avg_ang_vel = vector3_t::Zero();

            for (const auto& obs : observations_)
            {
                avg_ang_vel += obs.state.segment(6, 3); // assuming angular velocity is in the state vector
                avg_lin_vel += obs.state.segment(9, 3); // assuming linear velocity is in the state vector
            }
            avg_lin_vel /= observations_.size();
            avg_ang_vel /= observations_.size();

            // RCLCPP_INFO(this->get_logger(), "Average Linear Velocity: [%f, %f, %f]", avg_lin_vel(0), avg_lin_vel(1), avg_lin_vel(2));
            // RCLCPP_INFO(this->get_logger(), "Average Angular Velocity: [%f, %f, %f]", avg_ang_vel(0), avg_ang_vel(1), avg_ang_vel(2));

            vector3_t cmd_vel_lin = { latest_cmd_vel_.twist.linear.x,
                                      latest_cmd_vel_.twist.linear.y,
                                      latest_cmd_vel_.twist.linear.z };
            vector3_t cmd_vel_ang = { latest_cmd_vel_.twist.angular.x,
                                      latest_cmd_vel_.twist.angular.y,
                                      latest_cmd_vel_.twist.angular.z };

            // RCLCPP_INFO(this->get_logger(), "Commanded Linear Velocity: [%f, %f, %f]", cmd_vel_lin(0), cmd_vel_lin(1), cmd_vel_lin(2));
            // RCLCPP_INFO(this->get_logger(), "Commanded Angular Velocity: [%f, %f, %f]", cmd_vel_ang(0), cmd_vel_ang(1), cmd_vel_ang(2));
            vector3_t lin_error = cmd_vel_lin - avg_lin_vel;
            vector3_t ang_error = cmd_vel_ang - avg_ang_vel;

            vector3_t lin_error_percentage = vector3_t::Zero();
            vector3_t ang_error_percentage = vector3_t::Zero();
            if (cmd_vel_lin.norm() > 0.0)
            {
                lin_error_percentage = lin_error.cwiseAbs() / cmd_vel_lin.norm();
            }

            if (cmd_vel_ang.norm() > 0.0)
            {
                ang_error_percentage = ang_error.cwiseAbs() / cmd_vel_ang.norm();
            }

            // only print three decimal places
            // lin_error = (lin_error * 1000.0).array().round() / 1000.0;
            // ang_error = (ang_error * 1000.0).array().round() / 1000.0;

            lin_error_percentage = (lin_error_percentage * 1000.0).array().round() / 1000.0;
            ang_error_percentage = (ang_error_percentage * 1000.0).array().round() / 1000.0;

            // RCLCPP_INFO(this->get_logger(), "Linear Velocity Error: [%f, %f, %f] (percentage: [%f%, %f%, %f%])",
            //             lin_error(0), lin_error(1), lin_error(2),
            //             lin_error_percentage(0) * 100.0, lin_error_percentage(1) * 100.0, lin_error_percentage(2) * 100.0);
            // RCLCPP_INFO(this->get_logger(), "Angular Velocity Error: [%f, %f, %f] (percentage: [%f%, %f%, %f%])",
            //             ang_error(0), ang_error(1), ang_error(2),
            //             ang_error_percentage(0) * 100.0, ang_error_percentage(1) * 100.0, ang_error_percentage(2) * 100.0);

        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        geometry_msgs::msg::TwistStamped latest_cmd_vel_;

        rclcpp::TimerBase::SharedPtr timer_{nullptr};

        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_sub_;
        SystemObservation latest_observation_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr latest_cmd_vel_pub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr torso_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr torso_vel_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pos_pub_;

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr FL_contact_force_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr FR_contact_force_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr RL_contact_force_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr RR_contact_force_pub_;

        std::vector<SystemObservation> observations_;

        std::mutex latest_cmd_vel_mutex_;
        std::mutex latest_observation_mutex_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<CmdVelScorer>();

    // RCLCPP_INFO(node->get_logger(), "CmdVelScorer Node instantiated.");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
};
//
// Created by Max on 12/12/2024.
//

#pragma once

#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_go2_commands/ReferenceExtrapolation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_core/misc/LoadData.h>

using namespace switched_model;

namespace ocs2 {
namespace quadruped {

class ReferenceTrajectoryGenerator {
    public:
        ReferenceTrajectoryGenerator(const rclcpp::Node::SharedPtr &node,
                                        const std::string &targetCommandFile)
        {
            node_ = node;
            targetCommandFile_ = targetCommandFile;

            defaultJointState_.setZero(12);
            loadSettings(targetCommandFile);

            // Subscribe to command velocity
            cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 1, std::bind(&ReferenceTrajectoryGenerator::cmdVelCallback, this, std::placeholders::_1));

            // Subscribe to observation
            observation_sub_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
                "/go2_mpc_observation", 1, std::bind(&ReferenceTrajectoryGenerator::observationCallback, this, std::placeholders::_1));

            // Initialize the reference trajectory publisher
            referencePublisher_ = node->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
                "/go2_mpc_target", 1);
        }

        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        void loadSettings(const std::string &targetCommandFile) 
        {
            // Load target COM height
            ocs2::loadData::loadCppDataType<scalar_t>(targetCommandFile, "comHeight", comHeight_);

            // Load default joint angles
            ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState_);
        }

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);
            latest_cmd_vel_ = *msg;
        }

        void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(latest_observation_mutex_);
            latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);

            firstObservationReceived_ = true;
        }

        void publishReferenceTrajectory()
        {
            // std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);
            // std::lock_guard<std::mutex> obs_lock(latest_observation_mutex_);

            if (!firstObservationReceived_) 
            {
                // RCLCPP_WARN_STREAM(node_->get_logger(), "No observation received yet. Cannot publish reference trajectory.");
                return;
            }

            // Generate reference trajectory
            ocs2::TargetTrajectories referenceTrajectory = generateReferenceTrajectory(node_->get_clock()->now().seconds(), 0.0);

            // Publish reference trajectory
            referencePublisher_->publish(ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(referenceTrajectory));
        }

        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        BaseReferenceHorizon getBaseReferenceHorizon() 
        {
            return {trajdt_, trajKnots_};
        }

        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        BaseReferenceState getBaseReferenceState() 
        {
            std::lock_guard<std::mutex> lock(latest_observation_mutex_);
            scalar_t observationTime = latest_observation_.time;
            Eigen::Vector3d positionInWorld = latest_observation_.state.segment<3>(3);
            Eigen::Vector3d eulerXyz = latest_observation_.state.head<3>();
            return {observationTime, positionInWorld, eulerXyz};
        }

        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        /*********************************************************************************************************************/
        BaseReferenceCommand getBaseReferenceCommand(scalar_t time) 
        {
            std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);

            return {latest_cmd_vel_.linear.x, latest_cmd_vel_.linear.y, latest_cmd_vel_.angular.z, comHeight_};
        }

        TerrainPlane getTerrainPlane() 
        {
            return TerrainPlane();
        }


        ocs2::TargetTrajectories generateReferenceTrajectory(scalar_t time, scalar_t dt) 
        {
            // Get base reference trajectory
            BaseReferenceTrajectory baseReferenceTrajectory = 
                                        generateExtrapolatedBaseReference(getBaseReferenceHorizon(), getBaseReferenceState(),
                                                                            getBaseReferenceCommand(time), getTerrainPlane());

            // Generate target trajectory
            ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
            const size_t N = desiredTimeTrajectory.size();
            ocs2::vector_array_t desiredStateTrajectory(N);
            ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));
            
            for (size_t i = 0; i < N; ++i) 
            {
                ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

                // base orientation
                state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

                auto Rt = switched_model::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

                // base position
                state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

                // base angular velocity
                state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

                // base linear velocity
                state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

                // joint angles
                state.segment<12>(12) = defaultJointState_;

                desiredStateTrajectory[i] = std::move(state);
            }

            ocs2::TargetTrajectories targetTrajectories = {std::move(desiredTimeTrajectory), 
                                                            std::move(desiredStateTrajectory), 
                                                            std::move(desiredInputTrajectory)};

            // RCLCPP_INFO_STREAM(node_->get_logger(), "Generating reference trajectory with " << N << " knots.");
            for (size_t i = 0; i < N; ++i) 
            {
                const auto& time = targetTrajectories.timeTrajectory[i];
                const auto& state = targetTrajectories.stateTrajectory[i];
                const auto& input = targetTrajectories.inputTrajectory[i];

                // RCLCPP_INFO_STREAM(node_->get_logger(), "   time[" << i << "]: " << time);
                // RCLCPP_INFO_STREAM(node_->get_logger(), "   state[" << i << "]");
                // RCLCPP_INFO_STREAM(node_->get_logger(), "         torso pose: " << state.head(6).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "         torso momentum: " << state.segment(6, 6).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 0: " << state.segment(12, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 1: " << state.segment(15, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 2: " << state.segment(18, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 3: " << state.segment(21, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "   input[" << i << "]: ");
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 0 contact force: " << input.segment(0, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 1 contact force: " << input.segment(3, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 2 contact force: " << input.segment(6, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 3 contact force: " << input.segment(9, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 0 joint velocity: " << input.segment(12, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 1 joint velocity: " << input.segment(15, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 2 joint velocity: " << input.segment(18, 3).transpose());
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 3 joint velocity: " << input.segment(21, 3).transpose());
            }

            return targetTrajectories;
        }        

    private:
        rclcpp::Node::SharedPtr node_;
        std::string targetCommandFile_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_sub_;
        scalar_t trajdt_ = 0.1;
        size_t trajKnots_ = 10;

        ocs2::vector_t defaultJointState_ = ocs2::vector_t::Zero(12); // Default joint angles, can be loaded from file
        scalar_t comHeight_ = 0.3; // Default COM height, can be loaded from file

        std::mutex latest_observation_mutex_;
        std::mutex latest_cmd_vel_mutex_;

        bool firstObservationReceived_ = false;
        geometry_msgs::msg::Twist latest_cmd_vel_;
        ocs2::SystemObservation latest_observation_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr referencePublisher_;
};

}  // namespace quadruped
}  // namespace ocs2
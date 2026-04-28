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
#include <ocs2_anymal_commands/ReferenceExtrapolation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"
#include "ocs2_switched_model_interface/terrain/PlaneFitting.h"
#include <ocs2_core/misc/LoadData.h>

using namespace switched_model;

namespace ocs2 {
namespace quadruped {

class LocalTerrainEstimator 
{
    public:
        LocalTerrainEstimator(const rclcpp::Node::SharedPtr &node);
        void updateFootholds(const ocs2::SystemObservation &observation);
        inline const TerrainPlane &getPlane() const { return terrainPlane_; }

    private:
        void updateLocalTerrainEstimate(const std::vector<vector3_t> &footholds);

        // Local terrain estimate
        TerrainPlane terrainPlane_;

        // last footholds
        std::vector<vector3_t> lastFootholds_;

        // Kinematics model
        std::unique_ptr<KinematicsModelBase<scalar_t>> kinematicsModel_;
};


class ReferenceTrajectoryGenerator 
{
    public:
        ReferenceTrajectoryGenerator(const rclcpp::Node::SharedPtr &node,
                                     const std::string &targetCommandFile);
        void publishReferenceTrajectory();

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void reset() { firstObservationReceived_ = false; }

        bool isInitialized() const { return firstObservationReceived_; }

    private:
        BaseReferenceHorizon getBaseReferenceHorizon();
        BaseReferenceCommand getBaseReferenceCommand(scalar_t time);
        BaseReferenceState getBaseReferenceState();
        TerrainPlane getTerrainPlane();
        ocs2::TargetTrajectories generateReferenceTrajectory(scalar_t time, scalar_t dt);
        void loadSettings(const std::string &targetCommandFile);
        void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg);

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

        std::shared_ptr<LocalTerrainEstimator> localTerrainEstimator_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr referencePublisher_;
};

}  // namespace quadruped
}  // namespace ocs2
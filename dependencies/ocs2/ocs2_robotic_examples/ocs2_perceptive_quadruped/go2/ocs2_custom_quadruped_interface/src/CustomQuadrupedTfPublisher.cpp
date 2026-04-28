//
// Created by rgrandia on 10.03.22.
//

#include "ocs2_custom_quadruped_interface/CustomQuadrupedTfPublisher.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// URDF stuff
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

namespace switched_model {

void CustomQuadrupedTfPublisher::launchNode(const rclcpp::Node::SharedPtr& node,
                                      std::vector<std::string> jointNames,
                                      std::string baseName,
                                      const std::string& tfPrefix) {
  node_ = node;
  tfPrefix_ = tfPrefix;
  jointNames_ = std::move(jointNames);
  baseName_ = std::move(baseName);
  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  jointPublisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  lastTimeStamp_ = node_->get_clock()->now();
}

void CustomQuadrupedTfPublisher::publish(rclcpp::Time timeStamp,
                                   const vector_t& state,
                                   const std::string& worldFrame) 
{
  publish(timeStamp, getBasePose(state), getJointPositions(state), worldFrame);
}

void CustomQuadrupedTfPublisher::publish(rclcpp::Time timeStamp,
                                          const base_coordinate_t& basePose,
                                          const joint_coordinate_t& jointPositions,
                                          const std::string& worldFrame) 
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[CustomQuadrupedTfPublisher::publish()]");

  if (jointPublisher_ != nullptr && lastTimeStamp_ != timeStamp) 
  {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing TF and Joint states");
    // Joint positions
    updateJointPositions(jointPositions);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = timeStamp;

    const auto joint_size = jointNames_.size();
    joint_state.name.reserve(joint_size);
    for (const auto& name : jointNames_) {
      joint_state.name.emplace_back(name);
    }
    joint_state.position.reserve(joint_size);
    for (const auto& jointPosition : jointPositions)
      joint_state.position.emplace_back(jointPosition);
    jointPublisher_->publish(joint_state);

    // Base positions
    updateBasePose(timeStamp, basePose, worldFrame);
    tfBroadcaster_->sendTransform(baseToWorldTransform_);

    lastTimeStamp_ = timeStamp;
  } 
  // else
  // {
  //   RCLCPP_INFO_STREAM(node_->get_logger(), "Skipping TF and Joint states publishing");
  //   if (jointPublisher_ != nullptr) 
  //   {
  //     RCLCPP_INFO_STREAM(node_->get_logger(), "Time stamp is the same: " << lastTimeStamp_.seconds() << " == " << timeStamp.seconds());
  //   } else 
  //   {
  //     RCLCPP_INFO_STREAM(node_->get_logger(), "Joint publisher is null.");
  //   }
  // }
}

void CustomQuadrupedTfPublisher::updateJointPositions(const joint_coordinate_t& jointPositions) {
  jointPositionsMap_[jointNames_[0]] = jointPositions[0];
  jointPositionsMap_[jointNames_[1]] = jointPositions[1];
  jointPositionsMap_[jointNames_[2]] = jointPositions[2];
  jointPositionsMap_[jointNames_[3]] = jointPositions[3];
  jointPositionsMap_[jointNames_[4]] = jointPositions[4];
  jointPositionsMap_[jointNames_[5]] = jointPositions[5];
  jointPositionsMap_[jointNames_[6]] = jointPositions[6];
  jointPositionsMap_[jointNames_[7]] = jointPositions[7];
  jointPositionsMap_[jointNames_[8]] = jointPositions[8];
  jointPositionsMap_[jointNames_[9]] = jointPositions[9];
  jointPositionsMap_[jointNames_[10]] = jointPositions[10];
  jointPositionsMap_[jointNames_[11]] = jointPositions[11];
}

void CustomQuadrupedTfPublisher::updateBasePose(rclcpp::Time timeStamp,
                                          const base_coordinate_t& basePose,
                                          const std::string& worldFrame) {
  baseToWorldTransform_.header = ocs2::getHeaderMsg(worldFrame, timeStamp);
  baseToWorldTransform_.child_frame_id = tfPrefix_ + "/" + baseName_;

  const Eigen::Quaternion<scalar_t> q_world_base =
      quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  baseToWorldTransform_.transform.rotation =
      ocs2::getOrientationMsg(q_world_base);
  baseToWorldTransform_.transform.translation =
      ocs2::getVectorMsg(getPositionInOrigin(basePose));
}

}  // namespace switched_model
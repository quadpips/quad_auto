//
// Created by Max on 12/12/2024.
//

#pragma once

#include <mutex>
// #include <ros/subscriber.h>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

using namespace switched_model;

namespace ocs2 {
namespace quadruped {

class TargetTrajectoriesPublisher final
{
public:
  using CmdToTargetTrajectories =
      std::function<TargetTrajectories(const geometry_msgs::msg::Twist& cmd_vel, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(const rclcpp::Node::SharedPtr & node, const std::string& topic_prefix,
                              CmdToTargetTrajectories cmd_vel_to_target_trajectories)
    : cmd_vel_to_target_trajectories_(std::move(cmd_vel_to_target_trajectories))
    // , tf2_(buffer_)
  {
    // Trajectories publisher
    target_trajectories_publisher_.reset(new TargetTrajectoriesRosPublisher(node, topic_prefix));

    auto observation_callback = [this](const ocs2_msgs::msg::MpcObservation& msg) 
    {
      std::lock_guard<std::mutex> lock(latest_observation_mutex_);
      latest_observation_ = ros_msg_conversions::readObservationMsg(msg);
    };

    observation_sub_ =
        node->create_subscription<ocs2_msgs::msg::MpcObservation>(topic_prefix + "_mpc_observation", 1, observation_callback);

    // cmd_vel subscriber
    auto cmd_vel_callback = [this](const geometry_msgs::msg::Twist& msg) 
    {
      // std::cout << "[cmd_vel_callback]" << std::endl;
      
      // if (latest_observation_.time == 0.0)
      //   return;

      // std::cout << "Twist: " << std::endl;
      // std::cout << "    linear: " << std::endl;
      // std::cout << "        x: " << msg->linear.x << std::endl;
      // std::cout << "        y: " << msg->linear.y << std::endl;
      // std::cout << "        z: " << msg->linear.z << std::endl;
      // std::cout << "    angular: " << std::endl;
      // std::cout << "        x: " << msg->angular.x << std::endl;
      // std::cout << "        y: " << msg->angular.y << std::endl;
      // std::cout << "        z: " << msg->angular.z << std::endl;


      // vector_t cmd_vel = vector_t::Zero(4);
      // cmd_vel[0] = msg->linear.x;
      // cmd_vel[1] = msg->linear.y;
      // cmd_vel[2] = msg->linear.z;
      // cmd_vel[3] = msg->angular.z;

      latest_cmd_vel_ = msg;
      new_cmd_vel = true;
    };

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, cmd_vel_callback);
  }



  void send_target_trajectories()
  {
    if (latest_observation_.time == 0.0 || !new_cmd_vel)
      return;

    // only send new trajectory if the robot is in stance mode
    // if (latest_observation_.mode != switched_model::ModeNumber::STANCE)
    //   return;

    // latest_cmd_vel_.linear.x = -0.05;

    // std::cout << "[send_target_trajectories] latest_cmd_vel: " << std::endl;
    // std::cout << "    linear: " << std::endl;
    // std::cout << "        x: " << latest_cmd_vel_.linear.x << std::endl;
    // std::cout << "        y: " << latest_cmd_vel_.linear.y << std::endl;
    // std::cout << "        z: " << latest_cmd_vel_.linear.z << std::endl;
    // std::cout << "    angular: " << std::endl;
    // std::cout << "        x: " << latest_cmd_vel_.angular.x << std::endl;
    // std::cout << "        y: " << latest_cmd_vel_.angular.y << std::endl;
    // std::cout << "        z: " << latest_cmd_vel_.angular.z << std::endl;

    const auto trajectories = cmd_vel_to_target_trajectories_(latest_cmd_vel_, latest_observation_);
    target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    new_cmd_vel = true;
  }

private:
  CmdToTargetTrajectories cmd_vel_to_target_trajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> target_trajectories_publisher_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // tf2_ros::Buffer buffer_;
  // tf2_ros::TransformListener tf2_;

  geometry_msgs::msg::Twist latest_cmd_vel_;

  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;

  bool new_cmd_vel = false;
};

}  // namespace quadruped
}  // namespace legged
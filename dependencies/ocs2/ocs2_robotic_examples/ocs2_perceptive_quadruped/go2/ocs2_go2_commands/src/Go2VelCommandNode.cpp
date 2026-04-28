//
// Created by qiayuan on 2022/7/24.
//

#include "ocs2_go2_commands/Go2VelCommandNode.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <angles/angles.h>

using namespace ocs2;
using namespace quadruped;

namespace
{
// scalar_t TARGET_DISPLACEMENT_VELOCITY = 0.25;
// scalar_t TARGET_ROTATION_VELOCITY = 0.25;
scalar_t COM_HEIGHT = 0.0; // Overridden by config file
vector_t INITIAL_JOINT_STATE(12);
scalar_t TIME_TO_TARGET = 0.0; // Overridden by config file
}  // namespace

// scalar_t estimateTimeToTarget(const vector_t& desired_base_displacement)
// {
//   const scalar_t& dx = desired_base_displacement(0);
//   const scalar_t& dy = desired_base_displacement(1);
//   const scalar_t& dyaw = desired_base_displacement(3);
//   const scalar_t rotation_time = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
//   const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
//   const scalar_t displacement_time = displacement / TARGET_DISPLACEMENT_VELOCITY;
//   return std::max(rotation_time, displacement_time);
// }

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& target_pose, const SystemObservation& observation)
{
  // ROS_INFO_STREAM("[targetPoseToTargetTrajectories]");

  // desired time trajectory
  const scalar_array_t time_trajectory{ observation.time, observation.time + TIME_TO_TARGET };

  // desired state trajectory
  // vector_t current_pose = observation.state.head(6);

  // current_pose(0) = 0.0; // zeroing out roll
  // current_pose(1) = 0.0; // zeroing out pitch
  // /////////// leaving yaw //////////////
  // //////////// leaving x //////////////
  // //////////// leaving y //////////////
  // current_pose(5) = COM_HEIGHT;

  // ROS_INFO_STREAM("   current_pose: " << current_pose.transpose());

  vector_array_t state_trajectory(2, vector_t::Zero(observation.state.size()));
  state_trajectory[0] = observation.state; // observation.state.head(12), INITIAL_JOINT_STATE; // just the current state
  state_trajectory[1] << target_pose,  vector_t::Zero(6), INITIAL_JOINT_STATE;

  // ROS_INFO_STREAM("   current state: ");
  // ROS_INFO_STREAM("         torso pose: " << state_trajectory[0].head(6).transpose());
  // ROS_INFO_STREAM("         torso momentum: " << state_trajectory[0].segment(6, 6).transpose());
  // ROS_INFO_STREAM("         leg 0: " << state_trajectory[0].segment(12,3).transpose());
  // ROS_INFO_STREAM("         leg 1: " << state_trajectory[0].segment(15,3).transpose());
  // ROS_INFO_STREAM("         leg 2: " << state_trajectory[0].segment(18,3).transpose());
  // ROS_INFO_STREAM("         leg 3: " << state_trajectory[0].segment(21,3).transpose());

  // ROS_INFO_STREAM("   target state: ");
  // ROS_INFO_STREAM("         torso pose: " << state_trajectory[1].head(6).transpose());
  // ROS_INFO_STREAM("         torso momentum: " << state_trajectory[1].segment(6, 6).transpose());
  // ROS_INFO_STREAM("         leg 0: " << state_trajectory[1].segment(12,3).transpose());
  // ROS_INFO_STREAM("         leg 1: " << state_trajectory[1].segment(15,3).transpose());
  // ROS_INFO_STREAM("         leg 2: " << state_trajectory[1].segment(18,3).transpose());
  // ROS_INFO_STREAM("         leg 3: " << state_trajectory[1].segment(21,3).transpose());

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t input_trajectory(2, vector_t::Zero(observation.input.size()));

  // ocs2::TargetTrajectories targetTrajectories = { time_trajectory, state_trajectory, input_trajectory };
  // std::cout << "Generating reference trajectory with " << targetTrajectories.timeTrajectory.size() << " knots." << std::endl;
  // for (size_t i = 0; i < targetTrajectories.timeTrajectory.size(); ++i) 
  // {
  //     const auto& time = targetTrajectories.timeTrajectory[i];
  //     const auto& state = targetTrajectories.stateTrajectory[i];
  //     const auto& input = targetTrajectories.inputTrajectory[i];

  //     std::cout << "   time[" << i << "]: " << time << std::endl;
  //     std::cout << "   state[" << i << "]" << std::endl;
  //     std::cout << "         torso pose: " << state.head(6).transpose() << std::endl;
  //     std::cout << "         torso momentum: " << state.segment(6, 6).transpose() << std::endl;
  //     std::cout << "         leg 0: " << state.segment(12, 3).transpose() << std::endl;
  //     std::cout << "         leg 1: " << state.segment(15, 3).transpose() << std::endl;
  //     std::cout << "         leg 2: " << state.segment(18, 3).transpose() << std::endl;
  //     std::cout << "         leg 3: " << state.segment(21, 3).transpose() << std::endl;
  //     std::cout << "   input[" << i << "]: " << std::endl;
  //     std::cout << "       leg 0 contact force: " << input.segment(0, 3).transpose() << std::endl;
  //     std::cout << "       leg 1 contact force: " << input.segment(3, 3).transpose() << std::endl;
  //     std::cout << "       leg 2 contact force: " << input.segment(6, 3).transpose() << std::endl;
  //     std::cout << "       leg 3 contact force: " << input.segment(9, 3).transpose() << std::endl;
  //     std::cout << "       leg 0 joint velocity: " << input.segment(12, 3).transpose() << std::endl;
  //     std::cout << "       leg 1 joint velocity: " << input.segment(15, 3).transpose() << std::endl;
  //     std::cout << "       leg 2 joint velocity: " << input.segment(18, 3).transpose() << std::endl;
  //     std::cout << "       leg 3 joint velocity: " << input.segment(21, 3).transpose() << std::endl;
  // }

  return { time_trajectory, state_trajectory, input_trajectory };
}

TargetTrajectories cmdVelToTargetTrajectories(const geometry_msgs::msg::Twist& cmd_vel, const SystemObservation& observation)
{
  // ROS_INFO_STREAM("[cmdVelToTargetTrajectories]");

  // ROS_INFO_STREAM("cmd_vel: ");
  // ROS_INFO_STREAM("    linear: ");
  // ROS_INFO_STREAM("        x: " << cmd_vel.linear.x);
  // ROS_INFO_STREAM("        y: " << cmd_vel.linear.y);
  // ROS_INFO_STREAM("        z: " << cmd_vel.linear.z);
  // ROS_INFO_STREAM("    angular: ");
  // ROS_INFO_STREAM("        x: " << cmd_vel.angular.x);
  // ROS_INFO_STREAM("        y: " << cmd_vel.angular.y);
  // ROS_INFO_STREAM("        z: " << cmd_vel.angular.z);

  vector_t current_pose = observation.state.head(6);

  const Eigen::Matrix<scalar_t, 3, 1> torso_orientation = current_pose.head(3);
  const Eigen::Matrix<scalar_t, 3, 1> torso_position = current_pose.tail(3);

  // ROS_INFO_STREAM("      current_pose: ");
  // ROS_INFO_STREAM("           orientation: " << torso_orientation.transpose());
  // ROS_INFO_STREAM("           position:    " << torso_position.transpose());

  Eigen::Matrix<scalar_t, 3, 1> cmd_vel_linear(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);

  vector_t cmd_vel_linear_rot = getRotationMatrixFromXyzEulerAngles(torso_orientation) * cmd_vel_linear;

  // const scalar_t time_to_target = TIME_TO_TARGET;

  const vector_t target_pose = [&]() 
  {
    vector_t target(6);
    target(0) = 0;                                                        // roll
    target(1) = 0;                                                        // pitch                
    target(2) = current_pose(2) + cmd_vel.angular.z * TIME_TO_TARGET;     // yaw

    target(3) = current_pose(3) + cmd_vel_linear_rot(0) * TIME_TO_TARGET; // x
    target(4) = current_pose(4) + cmd_vel_linear_rot(1) * TIME_TO_TARGET; // y
    target(5) = COM_HEIGHT;                                               // z
    return target;
  }();

  // ROS_INFO_STREAM("      target_pose: ");
  // ROS_INFO_STREAM("           orientation: " << target_pose.head(3).transpose());
  // ROS_INFO_STREAM("           position:    " << target_pose.tail(3).transpose());

  // target reaching duration
  // const scalar_t target_reaching_time = observation.time + time_to_target;
  return targetPoseToTargetTrajectories(target_pose, observation);
}

int main(int argc, char* argv[])
{
  const std::string robot_name = "go2";

  // ros node handle
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robot_name + "_vel_command_node",
                                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                                .allow_undeclared_parameters(true));

  const std::string taskFile = node->get_parameter("taskFile").as_string();

  // loadData::loadCppDataType(reference_file, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  // loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);

  vector_t initial_robot_state(24);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialRobotState", initial_robot_state);

  COM_HEIGHT = initial_robot_state(5);

  INITIAL_JOINT_STATE = initial_robot_state.tail(12);

  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(node, 
                                                  robot_name, 
                                                  &cmdVelToTargetTrajectories);

  rclcpp::Rate rate(2.0);
  while (rclcpp::ok())
  {
    target_pose_command.send_target_trajectories();

    rclcpp::spin_some(node);
    rate.sleep();
  }

  // ros::spin();
  // Successful exit
  return 0;
}
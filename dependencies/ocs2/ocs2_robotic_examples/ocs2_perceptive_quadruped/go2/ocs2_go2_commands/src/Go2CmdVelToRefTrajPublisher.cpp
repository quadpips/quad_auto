#include <ocs2_go2_commands/Go2CmdVelToRefTrajPublisher.h>

using namespace ocs2;
using namespace quadruped;

int main(int argc, char* argv[])
{
  const std::string robot_name = "go2";

  // ros node handle
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robot_name + "_cmd_vel_to_ref_traj_node",
                                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                                .allow_undeclared_parameters(true));

  const std::string targetCommandFile = node->get_parameter("targetCommandFile").as_string();

  ReferenceTrajectoryGenerator ref_traj_gen(node, targetCommandFile);

  rclcpp::Rate rate(5.0);
  while (rclcpp::ok())
  {
    ref_traj_gen.publishReferenceTrajectory();

    rclcpp::spin_some(node);
    rate.sleep();
  }

  // ros::spin();
  // Successful exit
  return 0;
}
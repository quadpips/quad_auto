/*!
 * @file Go2GazeboBridge.hpp
 * @brief Setup data bridge between simulation and robot code.
 */

#pragma once

#include <go2_interface/Go2Bridge.hpp>

#include <unitree_go/msg/motor_states.hpp>
#include <unitree_go/msg/motor_cmds.hpp>

namespace legged_software {
namespace go2_interface {

class Go2GazeboBridge : public Go2Bridge
{
  public:
    Go2GazeboBridge(const rclcpp::Node::SharedPtr & node, Go2System* system);

    void lowCmdWrite() override;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void GroundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void jointStateCallback(const unitree_go::msg::MotorStates::SharedPtr msg);
    void CheaterModeCallback(const std::shared_ptr<go2_interface_msgs::srv::CheaterMode::Request> req,
                                    std::shared_ptr<go2_interface_msgs::srv::CheaterMode::Response> res);

  protected:
    rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr servo_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub;

    rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr servo_pub;
};

} // namespace go2_interface
} // namespace legged_software

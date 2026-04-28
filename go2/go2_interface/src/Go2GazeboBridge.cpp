/*!
 * @file Go2GazeboBridge.cpp
 * @brief Setup data bridge between simulation and robot code.
 */

#include "go2_interface/Go2GazeboBridge.hpp"

namespace legged_software {
namespace go2_interface {

/*!
 * Main method for GO2 Gazebo simulation
 */
Go2GazeboBridge::Go2GazeboBridge(const rclcpp::Node::SharedPtr &node, Go2System* system) : Go2Bridge(node, system)
{
    // Subscribers
    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/trunk_imu", 
        100, 
        std::bind(&Go2GazeboBridge::imuCallback, this, std::placeholders::_1)
    );
    ground_truth_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth/state", 
        100, 
        std::bind(&Go2GazeboBridge::GroundTruthCallback, this, std::placeholders::_1));
    servo_sub = node->create_subscription<unitree_go::msg::MotorStates>(
        "/unitree_joint_controller/state", 
        3, 
        std::bind(&Go2GazeboBridge::jointStateCallback, this, std::placeholders::_1));

    // Publishers
    servo_pub = node->create_publisher<unitree_go::msg::MotorCmds>("/unitree_joint_controller/command", 1);                                                                   

    // Initialize writing motor commands
    InitLowCmd();
}  

/*
*   Publish low-level commands to control joint motors
*/ 
void Go2GazeboBridge::lowCmdWrite()
{
    rclcpp::Time now = _node->get_clock()->now();
    rclcpp::Duration dt_loop = rclcpp::Duration::from_seconds(0);
    float des_dt = 1.0 / 500.0; // 500Hz

    while (rclcpp::ok())
    {
        now = _node->get_clock()->now();

        // Construct motor commands message
        unitree_go::msg::MotorCmds motor_cmds;
        motor_cmds.cmds.resize(12);
        for (int i = 0; i < 12; i++)
        {
            unitree_go::msg::MotorCmd &cmd = motor_cmds.cmds[i];
            cmd.q = low_cmd.motor_cmd[i].q;
            cmd.dq = low_cmd.motor_cmd[i].dq;
            cmd.tau = low_cmd.motor_cmd[i].tau;
            cmd.mode = low_cmd.motor_cmd[i].mode;
            cmd.kp = low_cmd.motor_cmd[i].kp;
            cmd.kd = low_cmd.motor_cmd[i].kd;
        }

        // Publish motor commands
        servo_pub->publish(motor_cmds);

        // Sleep to reach desired publish frequency
        dt_loop = _node->get_clock()->now() - now;
        if (dt_loop.seconds() < des_dt)
        {
            const rclcpp::Duration duration = rclcpp::Duration::from_seconds(des_dt - dt_loop.seconds());
            rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
        }
    }
}

/*
*   Process IMU readings
*/
void Go2GazeboBridge::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Save orientation measurements
    low_state.imu_state.quaternion[0] = msg->orientation.w;
    low_state.imu_state.quaternion[1] = msg->orientation.x;
    low_state.imu_state.quaternion[2] = msg->orientation.y;
    low_state.imu_state.quaternion[3] = msg->orientation.z;

    // Save gyroscope measurements
    low_state.imu_state.gyroscope[0] = msg->angular_velocity.x;
    low_state.imu_state.gyroscope[1] = msg->angular_velocity.y;
    low_state.imu_state.gyroscope[2] = msg->angular_velocity.z;

    // Save accelerometer measurements
    low_state.imu_state.accelerometer[0] = msg->linear_acceleration.x;
    low_state.imu_state.accelerometer[1] = msg->linear_acceleration.y;
    low_state.imu_state.accelerometer[2] = msg->linear_acceleration.z;
}

/*
*   Process joint state readings
*/
void Go2GazeboBridge::jointStateCallback(const unitree_go::msg::MotorStates::SharedPtr msg)
{
    // Save joint state readings
    for (int i = 0; i < 12; i++)
    {
        low_state.motor_state[i].mode = msg->states[i].mode;
        low_state.motor_state[i].q = msg->states[i].q;
        low_state.motor_state[i].dq = msg->states[i].dq;
        low_state.motor_state[i].tau_est = msg->states[i].tau_est;
    }
}

/*
*   Save ground-truth simulation data
*/
void Go2GazeboBridge::GroundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Indicate that data is currently in the world frame
    _cheaterState.use_world_frame = true;

    // Save position
    _cheaterState.position.x() = msg->pose.pose.position.x;
    _cheaterState.position.y() = msg->pose.pose.position.y;
    _cheaterState.position.z() = msg->pose.pose.position.z;

    // Save orientation
    _cheaterState.orientation.w() = msg->pose.pose.orientation.w;
    _cheaterState.orientation.x() = msg->pose.pose.orientation.x;
    _cheaterState.orientation.y() = msg->pose.pose.orientation.y;
    _cheaterState.orientation.z() = msg->pose.pose.orientation.z;

    // Save linear velocities
    _cheaterState.vWorld.x() = msg->twist.twist.linear.x;
    _cheaterState.vWorld.y() = msg->twist.twist.linear.y;
    _cheaterState.vWorld.z() = msg->twist.twist.linear.z;

    // Save angular velocities
    _cheaterState.omegaWorld.x() = msg->twist.twist.angular.x;
    _cheaterState.omegaWorld.y() = msg->twist.twist.angular.y;
    _cheaterState.omegaWorld.z() = msg->twist.twist.angular.z;

    // Save linear accelerations
    _cheaterState.aWorld.x() = low_state.imu_state.accelerometer[0];
    _cheaterState.aWorld.y() = low_state.imu_state.accelerometer[1];
    _cheaterState.aWorld.z() = low_state.imu_state.accelerometer[2];
}

} // namespace go2_interface
} // namespace legged_software

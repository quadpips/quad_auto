#include <iostream>
#include <go2_interface/Go2GazeboBridge.hpp>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

using legged_software::go2_interface::Go2GazeboBridge;
using legged_software::go2_interface::Go2System;

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        "go2_gazebo_interface",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO_STREAM(node->get_logger(), "GO2 Gazebo Interface Node started...");

    RCLCPP_INFO_STREAM(node->get_logger(), "Building Go2 system...");
    std::unique_ptr<Go2System> go2_sys = std::make_unique<Go2System>(node);

    RCLCPP_INFO_STREAM(node->get_logger(), "Building Go2 Gazebo bridge...");
    std::unique_ptr<Go2GazeboBridge> go2 = std::make_unique<Go2GazeboBridge>(node, go2_sys.get());

    RCLCPP_INFO_STREAM(node->get_logger(), "Done building Go2 Gazebo bridge!");

    go2->run();

    rclcpp::spin(node);
    rclcpp::shutdown();

    // why is this run after while loop?
    // go2->joinThreads();

    return 0;

}
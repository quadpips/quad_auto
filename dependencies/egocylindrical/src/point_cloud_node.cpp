//
// Created by root on 2/5/18.
//

#include <egocylindrical/point_cloud_generator.h>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv)
{
    // ros::init(argc, argv, "egocylindrical_pointcloud_publisher");
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        "egocylindrical_pointcloud_publisher",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");
    egocylindrical::EgoCylinderPointCloudGenerator s(node);
    s.init();

    rclcpp::spin(node);
}

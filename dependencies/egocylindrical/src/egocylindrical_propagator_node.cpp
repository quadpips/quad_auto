//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>

namespace egocylindrical
{


}



int main(int argc, char** argv)
{
    // ros::init(argc, argv, "egocylindrical_propagator");
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        "egocylindrical_propagator",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");

    egocylindrical::EgoCylindricalPropagator s(node);
    s.init();
    
    rclcpp::spin(node);
    // ros::MultiThreadedSpinner spinner;
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();
}

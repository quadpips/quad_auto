#include <egocylindrical/range_image_generator.h>

//Redundant
#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv)
{
    // ros::init(argc, argv, "egocylindrical_range_image_publisher");
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        "egocylindrical_range_image_publisher",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");
    egocylindrical::EgoCylinderRangeImageGenerator s(node);
    s.init();
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    rclcpp::spin(node);
}

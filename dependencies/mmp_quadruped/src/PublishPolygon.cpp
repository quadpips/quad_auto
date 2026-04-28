#include <mmp_quadruped/PolygonPublisher.h>

using namespace mmp;

int main(int argc, char* argv[])
{
  // ros::init(argc, argv, "polygon_publisher");
  // ros::NodeHandle nh;
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("polygon_publisher",
                                                              rclcpp::NodeOptions()
                                                              .allow_undeclared_parameters(true)
                                                              .automatically_declare_parameters_from_overrides(true));  
  
  std::string env_file = "N/A";
  node->get_parameter("envFile", env_file);

  if (env_file == "N/A")
  {
    RCLCPP_ERROR(node->get_logger(), "No environment file provided. Please provide a file path to the environment file.");
    return 0;
  }

  PolygonPublisher * polygonPublisher = new PolygonPublisher(node, env_file);

  rclcpp::Rate rate(10);

  while (rclcpp::ok())
  {
    polygonPublisher->publishLocalPolygons();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 1;
}
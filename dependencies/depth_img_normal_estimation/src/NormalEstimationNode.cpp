// #include <ros/init.h>
// #include <ros/node_handle.h>

#include <depth_img_normal_estimation/NormalEstimator.h>

// void ros_throw_if(const bool & condition, const std::string & message)
// {
//     if (condition)
//     {
//         ROS_ERROR_STREAM(message);
//         throw std::runtime_error(message);
//     }
// }

void ros_throw_param_load(const rclcpp::Node::SharedPtr & nodePtr, const std::string & param_name, std::string & param)
{
    // nodePtr->declare_parameter(param_name, "default");

    // if (nodePtr->has_parameter(param_name))
    // {
    param = nodePtr->get_parameter(param_name).as_string();
    // } else
    // {
        // throw std::runtime_error("Couldn't find parameter: " + param_name);
    // }

    // return ros_throw_if( !nh.getParam(param_name, param), "Couldn't find parameter: " + param_name);
}

void ros_throw_param_load(const rclcpp::Node::SharedPtr & nodePtr, const std::string & param_name, bool & param)
{
    // nodePtr->declare_parameter(param_name, false);

    // if (nodePtr->has_parameter(param_name))
    // {
    param = nodePtr->get_parameter(param_name).as_bool();
    // } else
    // {
        // throw std::runtime_error("Couldn't find parameter: " + param_name);
    // }    
    // return ros_throw_if( !nh.getParam(param_name, param), "Couldn't find parameter: " + param_name);
}

int main(int argc, char** argv) 
{
    // Initialize ros node
    // ros::init(argc, argv, "normal_estimation_node");
    // ros::NodeHandle nodeHandle;
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodePtr = rclcpp::Node::make_shared("depth_img_normal_estimation_node",
                                                                rclcpp::NodeOptions()
                                                                .allow_undeclared_parameters(true)
                                                                .automatically_declare_parameters_from_overrides(true));

    // ros::Rate loop_rate(30);
    rclcpp::Rate loop_rate(30);

    std::string camera_depth_topic;
    ros_throw_param_load(nodePtr, "camera_depth_topic", camera_depth_topic);

    std::string config_path;
    ros_throw_param_load(nodePtr, "config_path", config_path);

    bool hardware;
    ros_throw_param_load(nodePtr, "hardware", hardware);

    // Create NormalEstimator object
    NormalEstimator normalEstimator(nodePtr, camera_depth_topic, config_path, hardware);

    // ros::spin();
    rclcpp::spin(nodePtr);

    // while (ros::ok())
    // {
    //     // Do something
    //     normalEstimator.runNormalEstimation();
     
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }

}
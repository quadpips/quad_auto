#ifndef EGOCYLINDER_PARAMETER_GETTER_H
#define EGOCYLINDER_PARAMETER_GETTER_H

#include <string>
// #include <ros/node_handle.h>
// #include <ros/console.h>

#include <rclcpp/rclcpp.hpp>

namespace egocylindrical
{
  namespace utils
  {
      template <typename T>
      bool getParam(rclcpp::Node::SharedPtr node, std::string param_name, T& value) // ros::NodeHandle nh, 
      {
          if (node->get_parameter(param_name, value))
          {
              RCLCPP_INFO_STREAM(node->get_logger(), "Successfully loaded parameter [" << param_name << "]=" << value);
              return true;
          }
          else
          {
              RCLCPP_ERROR_STREAM(node->get_logger(), "Unable to find parameter [" << param_name << "]! Full namespace=" << node->get_namespace() << "/" << param_name);
              return false;
          }
      }

  }
}


#endif //EGOCYLINDER_PARAMETER_GETTER_H

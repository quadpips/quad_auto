
#include <egocylindrical/sensor.h>
#include <egocylindrical/parameter_getter.hpp>

//These includes are likely redundant, but included for completeness
#include <string>
// #include <ros/node_handle.h>
// #include <ros/console.h>

namespace egocylindrical
{
  namespace utils
  {
  
      bool SensorCharacteristics::init(rclcpp::Node::SharedPtr node) // ros::NodeHandle sensor_nh
      {
            //Find name of sensor
            // auto get_name = [node](std::string& value)
            // {
            //     std::string ns = node->get_namespace();
            //     auto pos = ns.rfind("/");

            //     if(pos == std::string::npos)
            //     {
            //         RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid namespace for sensor! [" << ns << "]");
            //         return false;
            //     }
                
            //     std::string substr = ns.substr(pos + 1);

            //     if(substr.empty())
            //     {
            //         RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid namespace for sensor! [" << ns << "]");
            //         return false;
            //     }
            //     value = substr;
            //     return true;
            // };

            auto get_param = [node](std::string param_name, auto& value)
            {
                /*
                if(sensor_nh.getParam(param_name, value))
                {
                    // ROS_INFO_STREAM("Successfully loaded parameter [" << param_name << "]=" << value);
                    return true;
                }
                else
                {
                    // // ROS_ERROR(_STREAM("Unable to find parameter [" << param_name << "]! Full namespace=" << sensor_nh.getNamespace() + "/" + param_name);
                    return false;
                }
                */
                return getParam(node, param_name, value);
            };
          
            // auto parameters_and_prefixes = node->list_parameters({}, 10);

            // for (auto & name : parameters_and_prefixes.names) {
            //     std::cout << "Parameter name: " << name << std::endl;
            // }
            // for (auto & prefix : parameters_and_prefixes.prefixes) {
            //     std::cout << "Parameter prefix: " << prefix << std::endl;
            // }

            //sensor_nh.getParam("publish_update", publish_update);
            //sensor_nh.getParam("raytrace", raytrace);
            bool publish_update_res = get_param("sensors.semantic_depth.publish_update", publish_update);
            bool raytrace_res = get_param("sensors.semantic_depth.raytrace", raytrace);
            if (publish_update_res && raytrace_res) // get_name(name) && 
            {
                return true;
            }
            return false;
      }

  } //end namespace utils
  
} //end namespace egocylindrical

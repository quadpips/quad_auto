#ifndef EGOCYLINDRICAL_SENSOR_H
#define EGOCYLINDRICAL_SENSOR_H

#include <egocylindrical/ecwrapper.h>
#include <std_msgs/msg/header.hpp>

#include <rclcpp/rclcpp.hpp>

namespace egocylindrical
{
  namespace utils
  {
    /*
    template <typename T>
    bool getParam(ros::NodeHandle nh, std::string param_name, T& value)
    {
        if(nh.getParam(param_name, value))
        {
            // ROS_INFO_STREAM("Successfully loaded parameter [" << param_name << "]=" << value);
            return true;
        }
        else
        {
            // // ROS_ERROR(_STREAM("Unable to find parameter [" << param_name << "]! Full namespace=" << sensor_nh.getNamespace() + "/" + param_name);
            return false;
        }
    }
    */
    
    struct SensorCharacteristics
    {
      std::string name;
      bool publish_update=false;
      bool raytrace=false;
      
      //This is really a factory method, should potentially be moved elsewhere
      virtual bool init(rclcpp::Node::SharedPtr node); // ros::NodeHandle sensor_nh
      /*
      {
          //Find name of sensor
          auto get_name = [sensor_nh](std::string& value)
          {
              std::string ns = sensor_nh.getNamespace();
              auto pos = x.rfind("/");

              if(pos == std::string::npos)
              {
                  // // ROS_ERROR(_STREAM("Invalid namespace for sensor! [" << ns << "]");
                  return false;
              }
              
              std::string substr = x.substr(pos + 1);

              if(substr.empty())
              {
                  // // ROS_ERROR(_STREAM("Invalid namespace for sensor! [" << ns << "]");
                  return false;
              }
              value = substr;
              return true;
          };
          
          auto get_param = [sensor_nh](std::string param_name, auto& value)
          {
              return getParam(sensor_nh, param_name, value);
          };
          
          //sensor_nh.getParam("publish_update", publish_update);
          //sensor_nh.getParam("raytrace", raytrace);
          if(get_name(name) && get_param("publish_update", publish_update) && get_param("raytrace", raytrace))
          {
              return true;
          }
          return false;
      }
      */
    };
    
    class SensorMeasurement : public SensorCharacteristics
    {
    public:
      SensorMeasurement(SensorCharacteristics sc, std_msgs::msg::Header header): 
        SensorCharacteristics(sc),
        header(header) 
        {}

      virtual void insert(ECWrapper& cylindrical_points) = 0;
      std_msgs::msg::Header getHeader() const {return header;}
      
    public:
       std_msgs::msg::Header header;
       
       using Ptr = std::shared_ptr<SensorMeasurement>;
       using ConstPtr = std::shared_ptr<const SensorMeasurement>;
    };
    
    
    class SensorInterface
    {
    public:
      using Callback = boost::function<void(SensorMeasurement::Ptr) > ;
      void setCallback(Callback cb) {cb_ = cb;}
      virtual void init(std::string fixed_frame_id)=0;
      
    protected:
      Callback cb_=0;
      
      SensorCharacteristics sc_;
      
    public:
      using Ptr = std::shared_ptr<SensorInterface>;
    };
    
    template<typename T>
    class TypedSensorInterface: public SensorInterface
    {
    public:
    };
    
    
  } //end namespace utils
} //end namespace egocylindrical

// #include <egocylindrical/sensor_filter.h> //NOTE: This is here to avoid a circular dependency issue

#endif //EGOCYLINDRICAL_SENSOR_H

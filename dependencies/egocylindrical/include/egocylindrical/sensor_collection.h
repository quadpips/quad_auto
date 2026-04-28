#ifndef EGOCYLINDRICAL_SENSOR_LOADER_H
#define EGOCYLINDRICAL_SENSOR_LOADER_H

//#include <functional> 
//#include <mutex>  //?

#include <egocylindrical/sensor.h>


// #include <ros/node_handle.h>
//#include <ros/console.h>

#include <tf2_ros/buffer.h>
//#include <image_transport/image_transport.hpp>



namespace egocylindrical
{
    namespace utils 
    {
        SensorInterface::Ptr createSensor(rclcpp::Node::SharedPtr node, std::string name); // ros::NodeHandle sensor_nh, 
        
        class SensorCollection
        {
            std::string fixed_frame_id_;
            
            // tf2_ros::Buffer& buffer_;
            std::shared_ptr<tf2_ros::Buffer> buffer_;

            // ros::NodeHandle pnh_;
            rclcpp::Node::SharedPtr node_;
            //i mage_transport::ImageTransport it_;

            std::vector<SensorInterface::Ptr> sensors_;
            
            using callback_t = const std::function <void (utils::SensorMeasurement::Ptr measurement)>;
            
        public:

            SensorCollection(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> buffer); // ros::NodeHandle pnh,

            bool init(std::string fixed_frame_id, callback_t& f);
            
            bool setup();
            
        };
    } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_SENSOR_LOADER_H

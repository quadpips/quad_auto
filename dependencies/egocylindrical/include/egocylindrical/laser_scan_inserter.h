#ifndef EGOCYLINDRICAL_LASER_SCAN_INSERTER_H
#define EGOCYLINDRICAL_LASER_SCAN_INSERTER_H

#include <egocylindrical/ecwrapper.h>

// #include <ros/node_handle.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/laser_scan.hpp>


namespace egocylindrical
{
    namespace utils
    {

        class LaserScanInserter
        {
            // tf2_ros::Buffer& buffer_;
            std::shared_ptr<tf2_ros::Buffer> buffer_;

            // ros::NodeHandle pnh_;
            rclcpp::Node::SharedPtr node_;
            std::string fixed_frame_id_;
          
        public:
            LaserScanInserter(std::shared_ptr<tf2_ros::Buffer> buffer, rclcpp::Node::SharedPtr node);
            
            bool init();
            
            bool init(std::string fixed_frame_id);
            
            bool insert(ECWrapper& cylindrical_points, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg);

        };
        
    } //end namespace utils
} //end namespace egocylindrical
#endif //EGOCYLINDRICAL_LASER_SCAN_INSERTER_H

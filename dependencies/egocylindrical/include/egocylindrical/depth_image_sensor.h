#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H

#include <egocylindrical/sensor.h>
#include <egocylindrical/depth_image_inserter.h>

#include <egocylindrical/time_filter.h>
#include <image_transport/subscriber_filter.hpp>
#include <tf2_ros/message_filter.h>
#include "tf2_ros/create_timer_ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std::placeholders;

namespace egocylindrical
{
  namespace utils
  {
    class DepthImageMeasurement: public SensorMeasurement
    {
    public:
      DepthImageMeasurement(SensorCharacteristics sc, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& image, 
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info, 
                            DepthImageInserter* dii):
        SensorMeasurement(sc, info->header),
        image_(image),
        info_(info),
        dii_(dii)
        {}
      
      //virtual std_msgs::msg::Header getHeader() const {return info_->header;}
      
      virtual void insert(ECWrapper& cylindrical_points)
      {
        // ros::WallTimetemp = ros::WallTime::now();
        dii_->insert(cylindrical_points, image_, info_); 
        // ROS_INFO_STREAM_NAMED("timing","Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
      }

      
    protected:
      const sensor_msgs::msg::Image::ConstSharedPtr image_;
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
      utils::DepthImageInserter* dii_;
    };

    class DepthImageSensor : public SensorInterface
    {
      // ros::NodeHandle pnh_;
      rclcpp::Node::SharedPtr node_;

      // tf2_ros::Buffer& buffer_;
      std::shared_ptr<tf2_ros::Buffer> buffer_;

      
      utils::DepthImageInserter dii_;
      
      image_transport::ImageTransport it_;
      image_transport::SubscriberFilter depth_sub_;
      message_filters::Subscriber<sensor_msgs::msg::CameraInfo> depth_info_sub_;

      using TimeFilter_t = TimeFilter<sensor_msgs::msg::CameraInfo>;
      std::shared_ptr<TimeFilter_t> time_filter_;
      
      using TfFilter = tf2_ros::MessageFilter<sensor_msgs::msg::CameraInfo>;
      std::shared_ptr<TfFilter> info_tf_filter;

      using MsgSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
      std::shared_ptr<MsgSynchronizer> msg_sync_;
      
    public:
      DepthImageSensor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> buffer):
        node_(node),
        buffer_(buffer),
        dii_(buffer, node),
        it_(node)
        {}

      
      void init(std::string fixed_frame_id) override
      {
        //Load general parameters
        sc_.init(node_);
        
        //sc_.name = depth_topic;
        //sc_.publish_update = true;
        //sc_.raytrace = true;
        
        //Load implementation parameters
        std::string depth_topic="/camera/depth/image_raw", 
                    info_topic= "/camera/depth/camera_info";
        // pnh_.getParam("image_in", depth_topic );
        // pnh_.getParam("info_in", info_topic );
        node_->get_parameter("image_in", depth_topic);
        node_->get_parameter("info_in", info_topic);
        
        //Initialize helper classes
        dii_.init(fixed_frame_id);
        
        //Set up publishers/subscribers and any necessary filters
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        qos.depth = 3; // TODO: Make this a parameter

        depth_sub_.subscribe(node_.get(), depth_topic, "raw", qos);
        depth_info_sub_.subscribe(node_.get(), info_topic, qos); // , 3

        // auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        //       node_->get_node_base_interface(),
        //       node_->get_node_timers_interface());
        // buffer_.setCreateTimerInterface(timer_interface);

        // Filter out images with duplicate time stamps
        // time_filter_ = std::make_shared<TimeFilter_t>(node_, depth_info_sub_);

        // Ensure that the message is transformable
        std::chrono::duration<int> buffer_timeout(1);
        info_tf_filter = std::make_shared<TfFilter>(depth_info_sub_, *buffer_, 
                                                    fixed_frame_id, 100, node_->get_node_logging_interface(),
                                                    node_->get_node_clock_interface(), buffer_timeout);

        // Synchronize Image and CameraInfo callbacks
        msg_sync_ = std::make_shared<MsgSynchronizer>(depth_sub_, *info_tf_filter, 10); //   
        msg_sync_->registerCallback(std::bind(&DepthImageSensor::update, this, _1, _2)); //  
      }
      
    protected:
      void update(const sensor_msgs::msg::Image::ConstSharedPtr& image, 
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) // , 
      {
        // Process the image and camera info messages here
        RCLCPP_INFO_STREAM(node_->get_logger(), "Received image (" << image->header.stamp.sec << "." << 
                                                                image->header.stamp.nanosec << 
                                                                ") and camera info at: " << 
                                                                info->header.stamp.sec << 
                                                                "." << info->header.stamp.nanosec << ")");
        if (cb_)
        {
          auto m = std::make_shared<DepthImageMeasurement>(sc_, image, info, &dii_);  
          cb_(m);
        }
        else
        {
          // ROS_ERROR(("No callback defined for DepthImageSensor!");
        }
      }
      
    public:
      using Ptr = std::shared_ptr<DepthImageSensor>;

    };
    
  } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H

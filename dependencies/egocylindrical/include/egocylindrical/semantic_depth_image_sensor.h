#ifndef EGOCYLINDRICAL_SEMANTIC_DEPTH_IMAGE_SENSOR_H
#define EGOCYLINDRICAL_SEMANTIC_DEPTH_IMAGE_SENSOR_H

#include <rclcpp/rclcpp.hpp>

#include <egocylindrical/sensor.h>
#include <egocylindrical/depth_image_sensor.h>
#include <egocylindrical/depth_image_inserter.h>

#include <egocylindrical/time_filter.h>
#include <image_transport/subscriber_filter.hpp>

#include <tf2_ros/message_filter.h>
#include "tf2_ros/create_timer_ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace egocylindrical
{
  namespace utils
  {
  class TerrainImageMeasurement: public SensorMeasurement
  {
  public:
    TerrainImageMeasurement(SensorCharacteristics sc, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& image, 
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& normals,
                            DepthImageInserter* dii):
      SensorMeasurement(sc, info->header),
      image_(image),
      info_(info),
      normals_(normals),
      dii_(dii)
      {}
    
    //virtual std_msgs::msg::Header getHeader() const {return info_->header;}
    
    virtual void insert(ECWrapper& cylindrical_points)
    {
      // ros::WallTimetemp = ros::WallTime::now();
      dii_->insert(cylindrical_points, image_, info_, normals_); //  
      // ROS_INFO_STREAM_NAMED("timing","Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
    }

  protected:
    const sensor_msgs::msg::Image::ConstSharedPtr image_;
    const sensor_msgs::msg::Image::ConstSharedPtr normals_;
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
    utils::DepthImageInserter* dii_;
  };

  // class SemanticDepthImageMeasurement: public SensorMeasurement
  // {
  // public:
  //   SemanticDepthImageMeasurement(SensorCharacteristics sc, 
  //                                 const sensor_msgs::msg::Image::ConstSharedPtr& image, 
  //                                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info, 
  //                                 const sensor_msgs::msg::Image::ConstSharedPtr& normals,
  //                                 const sensor_msgs::msg::Image::ConstSharedPtr& labels, 
  //                                 DepthImageInserter* dii):
  //     SensorMeasurement(sc, info->header),
  //     image_(image),
  //     info_(info),
  //     normals_(normals),
  //     labels_(labels),
  //     dii_(dii)
  //     {}
    
  //   //virtual std_msgs::msg::Header getHeader() const {return info_->header;}
    
  //   virtual void insert(ECWrapper& cylindrical_points)
  //   {
  //     // ros::WallTimetemp = ros::WallTime::now();
  //     dii_->insert(cylindrical_points, image_, info_, normals_, labels_); //  
  //     // ROS_INFO_STREAM_NAMED("timing","Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
  //   }

  // protected:
  //   const sensor_msgs::msg::Image::ConstSharedPtr image_;
  //   const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  //   const sensor_msgs::msg::Image::ConstSharedPtr normals_;
  //   const sensor_msgs::msg::Image::ConstSharedPtr labels_;
  //   utils::DepthImageInserter* dii_;
  // };

  class SemanticDepthImageSensor : public SensorInterface
  {
    // ros::NodeHandle pnh_;
    rclcpp::Node::SharedPtr node_;

    // tf2_ros::Buffer& buffer_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    
    utils::DepthImageInserter dii_;
    
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> depth_info_sub_;
    image_transport::SubscriberFilter normals_sub_;
    image_transport::Subscriber labels_sub_;

    using TimeFilter_t = TimeFilter<sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<TimeFilter_t> time_filter_;
    
    using TfFilter = tf2_ros::MessageFilter<sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<TfFilter> info_tf_filter;
    
    // sensor_msgs::msg::Image::ConstSharedPtr labels_;
    // bool new_labels = false;
    // bool have_labels = false;

    using MsgSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, 
                                                              sensor_msgs::msg::CameraInfo,
                                                              sensor_msgs::msg::Image>;
    std::shared_ptr<MsgSynchronizer> msg_sync_;

  public:
    SemanticDepthImageSensor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> buffer): // ros::NodeHandle pnh,
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
                  info_topic= "/camera/depth/camera_info",
                  normals_topic="/camera/normals",
                  labels_topic="/camera/steppability/labels";
      node_->get_parameter("image_in", depth_topic);
      node_->get_parameter("info_in", info_topic);
      node_->get_parameter("labels_in", labels_topic);
      node_->get_parameter("normals_in", normals_topic);

      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using depth topic: " << depth_topic);
      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using info topic: " << info_topic);
      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using normals topic: " << normals_topic);
      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using labels topic: " << labels_topic);

      //Initialize helper classes
      dii_.init(fixed_frame_id);
      
      //Set up publishers/subscribers and any necessary filters
      rmw_qos_profile_t qos = rmw_qos_profile_default;
      qos.depth = 3; // TODO: Make this a parameter

      depth_sub_.subscribe(node_.get(), depth_topic, "raw", qos);
      depth_info_sub_.subscribe(node_.get(), info_topic, qos); // , 3
      normals_sub_.subscribe(node_.get(), normals_topic, "raw", qos);

      // RCLCPP_INFO_STREAM_NAMED("update", "depth_topic: " << depth_topic);
      // RCLCPP_INFO_STREAM_NAMED("update", "info_topic: " << info_topic);
      // RCLCPP_INFO_STREAM_NAMED("update", "labels_topic: " << labels_topic);
      // RCLCPP_INFO_STREAM_NAMED("update", "normals_topic: " << normals_topic);

      // // Filter out images with duplicate time stamps
      // time_filter_ = std::make_shared<TimeFilter_t>(node_, depth_info_sub_);

      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using time filter with fixed frame ID: " << fixed_frame_id);
      
      // Ensure that the message is transformable
      // std::chrono::duration<int> buffer_timeout(1);
      // info_tf_filter = std::make_shared<TfFilter>(*time_filter_, *buffer_, 
      //                                             fixed_frame_id, 100, node_->get_node_logging_interface(),
      //                                             node_->get_node_clock_interface(), buffer_timeout);

      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using TF filter with fixed frame ID: " << fixed_frame_id);

      // Synchronize Image and CameraInfo callbacks
      msg_sync_ = std::make_shared<MsgSynchronizer>(depth_sub_, depth_info_sub_, normals_sub_, 10);
      msg_sync_->registerCallback(std::bind(&SemanticDepthImageSensor::update, this, _1, _2, _3));

      // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Using message synchronizer for depth, info, and normals topics");

      // labels_sub_ = it_.subscribe(labels_topic, 1, &SemanticDepthImageSensor::labels_cb, this);
    }
      
    protected:
      // void labels_cb(const sensor_msgs::msg::Image::ConstSharedPtr& labels)
      // {
      //   labels_ = labels;
      //   new_labels = true;
      //   have_labels = true;
      //   // // ROS_INFO_STREAM_NAMED("timing", "in solo callback, label: " << label);
      // }

      void update(const sensor_msgs::msg::Image::ConstSharedPtr& image, 
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                  const sensor_msgs::msg::Image::ConstSharedPtr& normals) //  
      {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Received image with timestamp: " << image->header.stamp.sec << "." << image->header.stamp.nanosec);
        // // ROS_INFO_STREAM_NAMED("timing", "labels->image.at<uint8_t>(50, 50): " << labels->image.at<uint8_t>(50, 50));

        // ROS_INFO_STREAM_NAMED("timing", "image timestamp: " << image->header.stamp);
        // ROS_INFO_STREAM_NAMED("timing", "info timestamp: " << info->header.stamp);
        // ROS_INFO_STREAM_NAMED("timing", "normals timestamp: " << normals->header.stamp);  

        if (cb_) //  && have_labels // want first update to be a depth+labels one
        {
          std::shared_ptr<SensorMeasurement> m;
          // if (new_labels)
          // {
          //   // ROS_INFO_STREAM_NAMED("timing", "image timestamp: " << image->header.stamp);
          //   // ROS_INFO_STREAM_NAMED("timing", "info timestamp: " << info->header.stamp);
          //   // ROS_INFO_STREAM_NAMED("timing", "labels_ timestamp: " << labels_->header.stamp);            
          //   m = std::make_shared<SemanticDepthImageMeasurement>(sc_, image, info, normals, labels_, &dii_); // normals,  
          //   new_labels = false;
          // } else
          // {
          // RCLCPP_INFO_STREAM(node_->get_logger(), "SemanticDepthImageSensor: Creating TerrainImageMeasurement");
          m = std::make_shared<TerrainImageMeasurement>(sc_, image, info, normals, &dii_); //    
          // }

          cb_(m);
        }
        else
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "No callback defined for SemanticDepthImageSensor!");
        }
      }
      
    public:
      using Ptr = std::shared_ptr<SemanticDepthImageSensor>;

    };
    
  } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H

#ifndef EGOCYLINDRICAL_LASER_SCAN_SENSOR_H
#define EGOCYLINDRICAL_LASER_SCAN_SENSOR_H

#include <egocylindrical/sensor.h>
#include <egocylindrical/laser_scan_inserter.h>

#include <egocylindrical/time_filter.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

namespace egocylindrical
{
  namespace utils
  {
    class LaserScanMeasurement: public SensorMeasurement
    {
    public:
      LaserScanMeasurement(SensorCharacteristics sc, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan, LaserScanInserter& lsi):
        SensorMeasurement(sc, scan->header),
        scan_(scan),
        lsi_(lsi)
        {}
      
      virtual void insert(ECWrapper& cylindrical_points)
      {
        // ros::WallTimetemp = ros::WallTime::now();
        lsi_.insert(cylindrical_points, scan_);
        // ROS_INFO_STREAM_NAMED("timing","Adding laser scan took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
      }

      
    protected:
      const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
      utils::LaserScanInserter& lsi_;
    };
    
    class LaserScanSensor: public SensorInterface
    {
      // ros::NodeHandle pnh_;
      rclcpp::Node::SharedPtr node_;

      // tf2_ros::Buffer& buffer_;
      std::shared_ptr<tf2_ros::Buffer> buffer_;

      
      LaserScanInserter lsi_;
      
      message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
      
      using TimeFilter_t = TimeFilter<sensor_msgs::msg::LaserScan>;
      std::shared_ptr<TimeFilter_t> time_filter_;
      
      using TfFilter = tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>;
      std::shared_ptr<TfFilter> scan_tf_filter;
      
    public:
      LaserScanSensor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> buffer):
        node_(node),
        buffer_(buffer),
        lsi_(buffer, node)
        {}

      
      void init(std::string fixed_frame_id) override
      {
        //Load general parameters
        sc_.init(node_);
        
        //sc_.name = scan_topic;
        //sc_.publish_update = true;
        //sc_.raytrace = true;
        
        //Load implementation parameters
        std::string scan_topic = "scan";
        // pnh_.getParam("scan_in", scan_topic);
        node_->get_parameter("scan_in", scan_topic);
        
        //Initialize helper classes
        lsi_.init(fixed_frame_id);
        
        //Set up publishers/subscribers and any necessary filters
        scan_sub_.subscribe(node_.get(), scan_topic); // , 3
        
        //Filter out images with duplicate time stamps
        time_filter_ = std::make_shared<TimeFilter_t>(node_, scan_sub_);

        // Ensure that the scan is transformable
        scan_tf_filter = std::make_shared<TfFilter>(*time_filter_, *buffer_, fixed_frame_id, 2, node_);

        scan_tf_filter->registerCallback(std::bind(&LaserScanSensor::update, this, _1));
      }
      
    protected:
      void update(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
      {
        if(cb_)
        {
          auto m = std::make_shared<LaserScanMeasurement>(sc_, scan, lsi_);
          cb_(m);
        }
        else
        {
          // ROS_ERROR(("No callback defined for LaserScanSensor!");
        }
      }
      
    public:
      using Ptr = std::shared_ptr<LaserScanSensor>;

    };
    
    
  } //end namespace utils
} //end namespace egocylindrical


#endif //EGOCYLINDRICAL_LASER_SCAN_SENSOR_H

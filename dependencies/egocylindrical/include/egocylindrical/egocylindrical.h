//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_EGOCYLINDRICAL_H
#define EGOCYLINDRICAL_EGOCYLINDRICAL_H

#include <egocylindrical/coordinate_frame_helper.h>


#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>


// #include <dynamic_reconfigure/server.h>
// #include <egocylindrical/PropagatorConfig.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#include <std_msgs/msg/empty.hpp>

//#include <egocylindrical/sensor.h>
//#include <egocylindrical/laser_scan_sensor.h>
//#include <egocylindrical/depth_image_sensor.h>
#include <egocylindrical/point_propagator.h>
//#include <egocylindrical/non_message_sequencer_filter.h>
#include <egocylindrical/sensor_collection.h>
#include <egocylindrical/ecwrapper_buffer.h>

namespace egocylindrical
{

class EgoCylindricalPropagator{


  
private:

    typedef boost::shared_mutex Mutex;
    typedef boost::unique_lock< Mutex > WriteLock;
    typedef boost::shared_lock< Mutex > ReadLock;

    Mutex config_mutex_, reset_mutex_;

    utils::ECWrapperPtr new_pts_, old_pts_, transformed_pts_, next_pts_;
    
    // ros::NodeHandle nh_, pnh_;
    rclcpp::Node::SharedPtr node_;

protected:
    // tf2_ros::Buffer buffer_;
    std::string fixed_frame_id_;
  
    std::shared_ptr<tf2_ros::Buffer> buffer_;

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::shared_ptr<utils::CoordinateFrameHelper> cfh_;

    //image_transport::ImageTransport it_;
    std::shared_ptr<utils::SensorCollection> sensors_;
    //utils::LaserScanSensor lss_;
    //utils::DepthImageSensor dis_;
    std::shared_ptr<utils::PointPropagator> pp_;
    ECWrapperBuffer wrapper_buffer_;

    rclcpp::Publisher<egocylindrical_msgs::msg::EgoCylinderPoints>::SharedPtr ec_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<egocylindrical_msgs::msg::EgoCylinderPoints>::SharedPtr info_pub_;
    rclcpp::Publisher<egocylindrical_msgs::msg::EgoCylinderPoints>::SharedPtr propagated_ec_pub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;

    std::chrono::steady_clock::time_point totalBegin;
    std::chrono::steady_clock::time_point totalEnd;
    float totalTimeTaken = 0.0f;
    int numberOfTotalCalls = 0;

    std::chrono::steady_clock::time_point updateBegin;
    std::chrono::steady_clock::time_point updateEnd;
    float updateTimeTaken = 0.0f;
    int numberOfUpdateCalls = 0;

    std::chrono::steady_clock::time_point insertBegin;
    std::chrono::steady_clock::time_point insertEnd;
    float insertTimeTaken = 0.0f;
    int numberOfInsertCalls = 0;

    std::chrono::steady_clock::time_point initStartTime;

    //egocylindrical::TimeSequencer<utils::SensorMeasurement> seq_;

    
    // egocylindrical::PropagatorConfig config_;
    // typedef dynamic_reconfigure::Server<egocylindrical::PropagatorConfig> ReconfigureServer;
    // std::shared_ptr<ReconfigureServer> reconfigure_server_;
    
    bool should_reset_;


    void propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::msg::Header new_header);
    
    void connectCB();
    
    // void configCB(const egocylindrical::PropagatorConfig &config, uint32_t level);
    
    virtual bool shouldPublish(const utils::ECWrapperPtr& points) { return true;}
    virtual void published(utils::ECWrapperPtr& points) { }


    
public:
    EgoCylindricalPropagator(rclcpp::Node::SharedPtr node);
    ~EgoCylindricalPropagator();
    
    void update(utils::SensorMeasurement& measurement);
    sensor_msgs::msg::PointCloud2  getPropagatedPointCloud();
    sensor_msgs::msg::Image::ConstSharedPtr getRawRangeImage();

    virtual bool init();
    void reset();

    void log();

};


} //end namespace egocylindrical


#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H

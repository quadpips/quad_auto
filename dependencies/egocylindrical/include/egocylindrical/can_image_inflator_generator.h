#ifndef EGOCYLINDRICAL_CAN_IMAGE_INFLATOR_GENERATOR_H
#define EGOCYLINDRICAL_CAN_IMAGE_INFLATOR_GENERATOR_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <egocylindrical/RangeImageInflatorGeneratorConfig.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

// #include <dynamic_reconfigure/server.h>

// #include <boost/thread/shared_mutex.hpp>
// #include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>




namespace egocylindrical
{


    class CanImageInflatorGenerator
    {
//         using Mutex = boost::mutex;
//         Mutex connect_mutex_;
//         using Lock = Mutex::scoped_lock;
        using Mutex = boost::mutex;
        using Lock = Mutex::scoped_lock;
        
        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher im_pub_;
        //ros::Subscriber range_img_sub_;
        image_transport::SubscriberFilter im_sub_;
        message_filters::Subscriber<EgoCylinderPoints> ec_sub_;
        
        typedef message_filters::TimeSynchronizer<sensor_msgs::msg::Image, egocylindrical_msgs::msg::EgoCylinderPoints> synchronizer;
        std::shared_ptr<synchronizer> timeSynchronizer_;
        
        Mutex connect_mutex_, config_mutex_;
        
        sensor_msgs::msg::Image::SharedPtr preallocated_msg_;
        
        typedef egocylindrical::RangeImageInflatorGeneratorConfig ConfigType;
        ConfigType config_;
        typedef dynamic_reconfigure::Server<ConfigType> ReconfigureServer;
        std::shared_ptr<ReconfigureServer> reconfigure_server_;

    public:

        CanImageInflatorGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        
        bool init();
        
        void configCB(const ConfigType &config, uint32_t level);
        
        void ssCB();



    private:
        
        void imgCB(const sensor_msgs::msg::Image::ConstSharedPtr& range_msg, const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg);
      
    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_CAN_IMAGE_INFLATOR_GENERATOR_H

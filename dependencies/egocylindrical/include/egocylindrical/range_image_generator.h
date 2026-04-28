#ifndef EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H
#define EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
// #include <egocylindrical/RangeImageGeneratorConfig.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <dynamic_reconfigure/server.h>

//#include <boost/thread/shared_mutex.hpp>
//#include <boost/thread/locks.hpp>

// typedef boost::shared_mutex Mutex;
// typedef boost::unique_lock< Mutex > WriteLock;
// typedef boost::shared_lock< Mutex > ReadLock;
#include <boost/thread/mutex.hpp>



namespace egocylindrical
{


    class EgoCylinderRangeImageGenerator
    {
        // ros::NodeHandle nh_, pnh_;
        rclcpp::Node::SharedPtr node_;

        image_transport::ImageTransport it_;
        image_transport::Publisher im_pub_, can_im_pub_;

        // ros::Subscriber ec_sub_;
        rclcpp::Subscription<egocylindrical_msgs::msg::EgoCylinderPoints>::SharedPtr ec_sub_;

        bool use_raw_;
        
        using Mutex = boost::mutex;
        Mutex connect_mutex_;
        using Lock = Mutex::scoped_lock;
        
        //Mutex config_mutex_;
        int num_threads_ = 1;

        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        
        sensor_msgs::msg::Image::SharedPtr preallocated_msg_, preallocated_can_msg_;
        
        // typedef egocylindrical::RangeImageGeneratorConfig ConfigType;
        // ConfigType config_;
        // typedef dynamic_reconfigure::Server<ConfigType> ReconfigureServer;
        // std::shared_ptr<ReconfigureServer> reconfigure_server_;

    public:

        // ros::NodeHandle& nh, ros::NodeHandle& pnh
        EgoCylinderRangeImageGenerator(rclcpp::Node::SharedPtr node);
        
        bool init();
        
        // void configCB(const ConfigType &config, uint32_t level);
        
        void ssCB();



    private:
        
        void ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H

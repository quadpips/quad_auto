#ifndef EGOCYLINDRICAL_FLOOR_IMAGE_GENERATOR_H
#define EGOCYLINDRICAL_FLOOR_IMAGE_GENERATOR_H

#include <chrono>
#include <fstream>
#include <string>

//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
// #include <egocylindrical/FloorImageGeneratorConfig.h>
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


    class EgoCylinderFloorImageGenerator
    {
        // ros::NodeHandle nh_, pnh_;
        rclcpp::Node::SharedPtr node_;

        image_transport::ImageTransport it_;
        image_transport::Publisher floor_im_pub_, labels_im_pub_, labels_colored_im_pub_, normals_im_pub_, normals_colored_im_pub_;
        // ros::Subscriber ec_sub_;
        rclcpp::Subscription<egocylindrical_msgs::msg::EgoCylinderPoints>::SharedPtr ec_sub_;
        bool use_raw_;

        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        
        // using Mutex = boost::mutex;
        // Mutex connect_mutex_;
        // using Lock = Mutex::scoped_lock;
        
        //Mutex config_mutex_;
        int num_threads_ = 1;
        
        sensor_msgs::msg::Image::SharedPtr preallocated_can_msg_, preallocated_labels_msg_, preallocated_labels_colored_msg_, preallocated_normals_msg_, preallocated_normals_colored_msgs_;
        
        std::chrono::steady_clock::time_point totalBegin;
        std::chrono::steady_clock::time_point totalEnd;
        float totalTimeTaken = 0.0f;
        int numberOfTotalCalls = 0;

        std::chrono::steady_clock::time_point initStartTime;

        // typedef egocylindrical::FloorImageGeneratorConfig ConfigType;
        // ConfigType config_;
        // typedef dynamic_reconfigure::Server<ConfigType> ReconfigureServer;
        // std::shared_ptr<ReconfigureServer> reconfigure_server_;
    public:

        EgoCylinderFloorImageGenerator(rclcpp::Node::SharedPtr node);
        
        bool init();

        void log();
        
        // void configCB(const ConfigType &config, uint32_t level);
        
        void ssCB();



    private:
        
        void ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H

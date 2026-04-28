#ifndef EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_H
#define EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

// #include <dynamic_reconfigure/server.h>

#include <rclcpp/rclcpp.hpp>
#include <boost/thread/mutex.hpp>


namespace egocylindrical
{


    class RangeImageDilator
    {
        using Mutex = boost::mutex;
        using Lock = Mutex::scoped_lock;

        // ros::NodeHandle nh_, pnh_;
        rclcpp::Node::SharedPtr node_;
        image_transport::ImageTransport it_;

        image_transport::SubscriberFilter im_sub_, can_im_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> ec_sub_;

        bool use_egocan_;

        typedef message_filters::TimeSynchronizer<sensor_msgs::msg::Image, egocylindrical_msgs::msg::EgoCylinderPoints> synchronizer;
        std::shared_ptr<synchronizer> timeSynchronizer;
        typedef message_filters::TimeSynchronizer<sensor_msgs::msg::Image, egocylindrical_msgs::msg::EgoCylinderPoints, sensor_msgs::msg::Image> can_synchronizer;
        std::shared_ptr<can_synchronizer> timeSynchronizerWithCan;

        // ros::Publisher im_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr im_pub_;

        Mutex connect_mutex_;

    public:

        RangeImageDilator(const rclcpp::Node::SharedPtr& node); // ros::NodeHandle& nh, ros::NodeHandle& pnh

        bool init();

        void ssCB();



    private:

        void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr& image, const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& info, const sensor_msgs::msg::Image::ConstSharedPtr& can_image);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_H


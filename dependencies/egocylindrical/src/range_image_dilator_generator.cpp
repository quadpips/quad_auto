
//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_dilator_generator.h>
// #include <egocylindrical/range_to_points.h>

// The below are redundant
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rmw/types.h"

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/range_image_dilator_core.h>

namespace egocylindrical
{

    RangeImageDilator::RangeImageDilator(const rclcpp::Node::SharedPtr& node) // ros::NodeHandle& nh, ros::NodeHandle& pnh
        : node_(node), // pnh_(pnh),
        // nh_(nh),
        // pnh_(pnh),
        it_(node_)
    {
        std::cout<<"Egocylindrical Range Image Dilator Node Initialized"<<std::endl;
    }

    // TODO: Is there any real need for the 'info' topic?
    bool RangeImageDilator::init()
    {
        use_egocan_ = false;
        // pnh_.getParam("egocan_enabled", use_egocan_);
        use_egocan_ = node_->get_parameter("egocan_enabled").as_bool();
        
        im_sub_.registerCallback(std::bind(&RangeImageDilator::imageCB, this, std::placeholders::_1, nullptr, nullptr));
        /*
        if(use_egocan_)
        {
          timeSynchronizerWithCan = std::make_shared<can_synchronizer>(im_sub_, ec_sub_, can_im_sub_, 20);
          timeSynchronizerWithCan->registerCallback(std::bind(&RangeImageDilator::imageCB, this, _1, _2, _3));

        }
        else
        {
          // Synchronize Image and CameraInfo callbacks
          timeSynchronizer = std::make_shared<synchronizer>(im_sub_, ec_sub_, 20);
          timeSynchronizer->registerCallback(std::bind(&RangeImageDilator::imageCB, this, _1, _2, nullptr));
        }
        */

        // ros::SubscriberStatusCallback info_cb = std::bind(&RangeImageDilator::ssCB, this);
        // {
            // Lock lock(connect_mutex_);
        im_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image_out", 2); // , info_cb, info_cb);
        // }

        return true;
    }

    void RangeImageDilator::ssCB()
    {

        //std::cout << (void*)ec_sub_ << ": " << im_pub_->get_subscription_count() << std::endl;
        Lock lock(connect_mutex_);
        if (im_pub_->get_subscription_count() > 0)
        {
            //Note: should probably add separate checks for each
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {

            }
            else
            {
                rmw_qos_profile_t qos = rmw_qos_profile_default;
                qos.depth = 2; // TODO: Make this a parameter
                im_sub_.subscribe(node_.get(), "image_in", "compressed", qos);

                if(use_egocan_)
                {
                    can_im_sub_.subscribe(node_.get(), "can_image_in", "compressed", qos);
                }
                ec_sub_.subscribe(node_.get(), "info_in");

                // ROS_INFO_STREAM("RangeImage Dilator Subscribing to [" << im_sub_.getTopic() << "]");
            }
        }
        else
        {
            im_sub_.unsubscribe();
            if(use_egocan_)
            {
                can_im_sub_.unsubscribe();
            }
            ec_sub_.unsubscribe();
            // ROS_INFO("RangeImage Dilator Unsubscribing");

        }
    }

    void RangeImageDilator::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr& image, const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& info, const sensor_msgs::msg::Image::ConstSharedPtr& can_image)
    {
        // ROS_DEBUG("Received range msg");

        // This may be redundant now
        if (im_pub_->get_subscription_count() > 0)
        {
            // ros::WallTime// start = ros::WallTime::now();

            auto dilated_img_ptr = utils::dilateImage(image);

            // ROS_DEBUG_STREAM("Dilating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");


            // ROS_DEBUG("publish egocylindrical image");

            im_pub_->publish(*dilated_img_ptr);
        }

    }

}

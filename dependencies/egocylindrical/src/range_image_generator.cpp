//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_generator.h>
#include <egocylindrical/range_image_core.h>
#include <egocylindrical/can_image_core.h>

using namespace std::chrono_literals;

namespace egocylindrical
{
    EgoCylinderRangeImageGenerator::EgoCylinderRangeImageGenerator(rclcpp::Node::SharedPtr node) :
        // nh_(nh),
        // pnh_(pnh),
        node_(node),
        it_(node)
    {
        std::cout<<"Egocylindrical Range Image Node Initialized"<<std::endl;

        timer_ = node_->create_wall_timer(0.05s, std::bind(&EgoCylinderRangeImageGenerator::ssCB, this));
    }
    
    bool EgoCylinderRangeImageGenerator::init()
    {
        use_raw_ = false;
        std::string image_topic = "image", can_image_topic = "can_image";
        
        // pnh_.getParam("use_raw", use_raw_ );
        node_->get_parameter("use_raw", use_raw_);
        
        // pnh_.getParam("image_topic", image_topic );
        node_->get_parameter("image_topic", image_topic);

        // pnh_.getParam("can_image_topic", can_image_topic );
        node_->get_parameter("can_image_topic", can_image_topic);
        
        // reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        // reconfigure_server_->setCallback(std::bind(&EgoCylinderRangeImageGenerator::configCB, this, _1, _2));

        // image_transport::SubscriberStatusCallback image_cb = std::bind(&EgoCylinderRangeImageGenerator::ssCB, this);
        // {
        //     Lock lock(connect_mutex_);
        im_pub_ = it_.advertise(image_topic, 2);
        can_im_pub_ = it_.advertise(can_image_topic, 2);
        // }
        
        return true;
    }
    
    // void EgoCylinderRangeImageGenerator::configCB(const ConfigType &config, uint32_t level)
    // {
    //   //Num_threads not actually used right now, so not important to lock
    //   //WriteLock lock(config_mutex_);
      
    //   // ROS_INFO_STREAM("Updating Range Image Generator config: num_threads=" << config.num_threads);
    //   num_threads_ = config.num_threads;
    // }
    
    void EgoCylinderRangeImageGenerator::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_->get_subscription_count() << std::endl;
        // Lock lock(connect_mutex_);
        if(im_pub_.getNumSubscribers() > 0 || can_im_pub_.getNumSubscribers() > 0)
        {
            if(ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                // ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderRangeImageGenerator::ecPointsCB, this);
                ec_sub_ = node_->create_subscription<egocylindrical_msgs::msg::EgoCylinderPoints>(
                    "egocylindrical_points", 2, std::bind(&EgoCylinderRangeImageGenerator::ecPointsCB, this, std::placeholders::_1));

                // ROS_INFO("RangeImage Generator Subscribing");

            }
      
        }
        else
        {
            ec_sub_.reset();
            // ROS_INFO("RangeImage Generator Unsubscribing");

        }
    }

    
    void EgoCylinderRangeImageGenerator::ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg)
    {
        // ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[range_image_generator] Received [" << ec_msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
        
        
        bool gen_range_image = im_pub_.getNumSubscribers() > 0;
        bool gen_can_image = can_im_pub_.getNumSubscribers() > 0;
        
        utils::ECWrapper ec_pts(ec_msg);
        
        if (gen_range_image)
        {
          // ros::WallTime// start = ros::WallTime::now();
                
          sensor_msgs::msg::Image::ConstSharedPtr image_ptr = use_raw_ ? utils::getRawRangeImageMsg(ec_pts, num_threads_, preallocated_msg_) : utils::getRangeImageMsg(ec_pts, num_threads_, preallocated_msg_);

          // ROS_DEBUG_STREAM_NAMED("timing","Generating egocylindrical range image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

          // ROS_DEBUG("publish egocylindrical image");
          
          im_pub_.publish(image_ptr);
          // ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[range_image_generator] Sent [" << image_ptr->header.stamp << "] at [" << ros::WallTime::now() << "]");
          
          // start = ros::WallTime::now();
          preallocated_msg_= std::make_shared<sensor_msgs::msg::Image>();
          preallocated_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          // ROS_DEBUG_STREAM_NAMED("timing","Preallocating range image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        }
        
        if(gen_can_image)
        {
          // ros::WallTime// start = ros::WallTime::now();
          
          sensor_msgs::msg::Image::ConstSharedPtr image_ptr = use_raw_ ? utils::getRawCanImageMsg(ec_pts, num_threads_, preallocated_msg_) : utils::getCanImageMsg(ec_pts, num_threads_, preallocated_msg_);
          
          // ROS_DEBUG_STREAM_NAMED("timing","Generating can image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          
          // ROS_DEBUG("publish egocylindrical image");
          
          can_im_pub_.publish(image_ptr);
          
          // start = ros::WallTime::now();
          preallocated_can_msg_= std::make_shared<sensor_msgs::msg::Image>();
          preallocated_can_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          // ROS_DEBUG_STREAM_NAMED("timing","Preallocating can image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        }
        
    }
}

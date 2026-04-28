//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_converter.h>
#include <egocylindrical/range_to_points.h>

// The below are redundant
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <egocylindrical/ecwrapper.h>


namespace egocylindrical
{


  // ros::NodeHandle& nh, ros::NodeHandle& pnh
    RangeImageConverter::RangeImageConverter(const rclcpp::Node::SharedPtr& node) :
        node_(node),  
        // nh_(nh),
        // pnh_(pnh),
        it_(node_)
    {
        std::cout<<"Egocylindrical Range Image Converter Node Initialized"<<std::endl;


    }
    
    bool RangeImageConverter::init()
    {
        use_egocan_ = false;
        // pnh_.getParam("egocan_enabled", use_egocan_);
        use_egocan_ = node_->get_parameter("egocan_enabled").as_bool();

        if (use_egocan_)
        {
          timeSynchronizerWithCan = std::make_shared<can_synchronizer>(im_sub_, ec_sub_, can_im_sub_, 20);
          timeSynchronizerWithCan->registerCallback(std::bind(&RangeImageConverter::imageCB, this, _1, _2, _3));
          
        }
        else
        {
          // Synchronize Image and CameraInfo callbacks
          timeSynchronizer = std::make_shared<synchronizer>(im_sub_, ec_sub_, 20);
          timeSynchronizer->registerCallback(std::bind(&RangeImageConverter::imageCB, this, _1, _2, nullptr));
        }
        
        // ros::SubscriberStatusCallback info_cb = std::bind(&RangeImageConverter::ssCB, this);
        // {
            // Lock lock(connect_mutex_);
            // ec_pub_ = nh_.advertise<egocylindrical_msgs::msg::EgoCylinderPoints>("data_out", 2, info_cb, info_cb);
        ec_pub_ = node_->create_publisher<egocylindrical_msgs::msg::EgoCylinderPoints>("data_out", 2); // , info_cb, info_cb);
        // }
        
        return true;
    }
    
    void RangeImageConverter::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_->get_subscription_count() << std::endl;
        Lock lock(connect_mutex_);
        if (ec_pub_->get_subscription_count() > 0)
        {
            //Note: should probably add separate checks for each
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
              // im_sub_.subscribe(it_, "image_in", 2);

              // if (use_egocan_)
                // can_im_sub_.subscribe(it_, "can_image_in", 2);
              // ec_sub_.subscribe(nh_, "info_in", 2);

              rmw_qos_profile_t qos = rmw_qos_profile_default;
              qos.depth = 2; // TODO: Make this a parameter
              im_sub_.subscribe(node_.get(), "image_in", "compressed", qos);              

              if (use_egocan_)
              {
                can_im_sub_.subscribe(node_.get(), "can_image_in", "compressed", qos);
              }
              ec_sub_.subscribe(node_.get(), "info_in");                
              
              // ROS_INFO("RangeImage Converter Subscribing");

            }
      
        }
        else
        {
            im_sub_.unsubscribe();
            if(use_egocan_)
              can_im_sub_.unsubscribe();
            ec_sub_.unsubscribe();
            // ROS_INFO("RangeImage Converter Unsubscribing");

        }
    }

    void RangeImageConverter::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr& image, 
                                      const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& info, 
                                      const sensor_msgs::msg::Image::ConstSharedPtr& can_image)
    {
        // ROS_DEBUG("Received range msg");
        
        // This may be redundant now
        if (ec_pub_->get_subscription_count() > 0)
        {
          // ros::WallTime// start = ros::WallTime::now();
          
          utils::ECWrapperPtr ec_pts = utils::range_image_to_wrapper(info, image, can_image);
          
          utils::ECMsgConstPtr ec_msg = ec_pts->getEgoCylinderPointsMsg();
          
                

          // ROS_DEBUG_STREAM("Converting egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          // ROS_DEBUG("publish generated egocylindrical data");
          
          ec_pub_->publish(*ec_msg);
        }
        
    }




}

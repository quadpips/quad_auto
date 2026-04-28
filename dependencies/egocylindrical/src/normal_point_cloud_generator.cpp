//
// Created by root on 2/5/18.
//

#include <egocylindrical/normal_point_cloud_generator.h>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace egocylindrical
{
    NormalPointCloudGenerator::NormalPointCloudGenerator(rclcpp::Node::SharedPtr node) :
        node_(node)
    {
        std::cout<<"Labeled PointCloud publishing Node Initialized"<<std::endl;

        timer_ = node_->create_wall_timer(0.05s, std::bind(&NormalPointCloudGenerator::ssCB, this));
    }
    
    bool NormalPointCloudGenerator::init()
    {
        ec_sub_.reset();
        
        // ros::SubscriberStatusCallback info_cb = std::bind(&NormalPointCloudGenerator::ssCB, this);
        // {
        //     Lock lock(connect_mutex_);
        pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("normals", 2);
        // }
        
        return true;
    }


    void NormalPointCloudGenerator::ssCB()
    {
        //std::cout << (void*)ec_sub_ << ": " << pc_pub_->get_subscription_count() << std::endl;
        Lock lock(connect_mutex_);
        if(pc_pub_->get_subscription_count()>0)
        {
            if(ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
              // ROS_INFO("Labeled PointCloud Generator Subscribing");
            //   ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &NormalPointCloudGenerator::ecPointsCB, this);
                ec_sub_ = node_->create_subscription<egocylindrical_msgs::msg::EgoCylinderPoints>("egocylindrical_points", 2, std::bind(&NormalPointCloudGenerator::ecPointsCB, this, std::placeholders::_1));
            }
      
        }
        else
        {
            ec_sub_.reset();
            // ROS_INFO("Labeled PointCloud Generator Unsubscribing");
        }
    }
    
    
    void NormalPointCloudGenerator::ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg)
    {
        // ROS_DEBUG("Received EgoCylinderPoints msg");

        if(pc_pub_->get_subscription_count()>0)
        {
          // ros::WallTime// start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
          
          sensor_msgs::msg::PointCloud2::ConstSharedPtr msg = utils::generate_normal_point_cloud(ec_pts);
          
          // ROS_DEBUG_STREAM("Generating labeled point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          // ROS_DEBUG("publish egocylindrical labeled pointcloud");
          
          pc_pub_->publish(*msg);
        }
        
    }




}

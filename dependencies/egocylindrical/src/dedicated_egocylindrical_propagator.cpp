//
// Created by root on 2/5/18.
//

#include "point_transformer.cpp"

#include <egocylindrical/dedicated_egocylindrical.h>
#include <egocylindrical/point_transformer.h>
#include <egocylindrical/depth_image_core.h>
#include <egocylindrical/range_image_core.h>

//#include <tf2/LinearMath/Quaternion.h>
//#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

//#include <valgrind/callgrind.h>

namespace egocylindrical
{

    
    namespace utils
    {
      
      void transformPoints(utils::ECWrapper& points, const utils::ECWrapper& new_points, const geometry_msgs::msg::TransformStamped& trans, int num_threads);
      
      //void transformPoints(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const geometry_msgs::msg::TransformStamped& trans, int num_threads=1);
      
    }
    

    void DedicatedEgoCylindricalPropagator::propagateHistoryInplace(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::msg::Header new_header)
    {
      // ros::WallTime// start = ros::WallTime::now();
      
      std_msgs::msg::Header old_header = old_pnts.getHeader();
      
      new_pnts.setHeader(new_header);
      
      // ROS_DEBUG("Getting Transformation details");
      geometry_msgs::msg::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                                                      old_header.frame_id, old_header.stamp,
                                                                      "odom");
      
      // ROS_DEBUG_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
      
      // start = ros::WallTime::now();    
      utils::transformPoints(old_pnts, new_pnts, trans, config_.num_threads);
      // ROS_DEBUG_STREAM_NAMED("timing", "Transforming points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
      // start = ros::WallTime::now();
      utils::addPoints(new_pnts, old_pnts, false);
      // ROS_DEBUG_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
    }


    void DedicatedEgoCylindricalPropagator::propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::msg::Header new_header)
    {
        // ros::WallTime// start = ros::WallTime::now();
        
        std_msgs::msg::Header old_header = old_pnts.getHeader();
        
        new_pnts.setHeader(new_header);
        
        // ROS_DEBUG("Getting Transformation details");
                geometry_msgs::msg::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                "odom");
        
        // ROS_DEBUG_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                
        
        // start = ros::WallTime::now();    
        utils::transformPoints(old_pnts, *transformed_pts_, new_pnts, trans, config_.num_threads);
        // ROS_DEBUG_STREAM_NAMED("timing", "Transforming points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        // start = ros::WallTime::now();
        utils::addPoints(new_pnts, *transformed_pts_, false);
        // ROS_DEBUG_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

    }


    void DedicatedEgoCylindricalPropagator::addDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
    {
        if(pc_pub_->get_subscription_count()>0)
        {
          ReadLock lock(config_mutex_);
          sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg;
          depth_remapper_.update(cylindrical_points, image, cam_info, pcloud_msg, config_.filter_y_min, config_.filter_y_max);
          pc_pub_.publish(pcloud_msg);
        }
        else
        {
            depth_remapper_.update(cylindrical_points, image, cam_info);
        }
    }


    void DedicatedEgoCylindricalPropagator::update(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
    {
        if(old_pts_ && old_pts_->getHeader().stamp >= cam_info->header.stamp)
        {
          old_pts_ = nullptr;
        }
        // ros::WallTime// start = ros::WallTime::now();
        
        //new_pts_ = utils::getECWrapper(cylinder_height_,cylinder_width_,vfov_);
        // NOTE: It may be better to only create the necessary wrappers once and just 'swap' the msg_ pointers
        new_pts_ = next_pts_;
        
        {
          {
            try
            {
                if(old_pts_)
                {
                    // ros::WallTime// start = ros::WallTime::now();
                    
                    DedicatedEgoCylindricalPropagator::propagateHistoryInplace(*old_pts_, *new_pts_, image->header);
                    //// ROS_INFO_STREAM("Propagation took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                }
                else
                {
                    new_pts_->setHeader(image->header);
                }
                
            }
            catch (tf2::TransformException &ex) 
            {
                // ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                return;
            }
            
            
            {
                // ros::WallTimetemp = ros::WallTime::now();
                DedicatedEgoCylindricalPropagator::addDepthImage(*new_pts_, image, cam_info);
                // ROS_DEBUG_STREAM("Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
            }
            
            if(im_pub_->get_subscription_count() > 0)
            {
                sensor_msgs::msg::Image::ConstSharedPtr image_ptr = use_raw_ ? utils::getRawRangeImageMsg(*new_pts_, 1) : utils::getRangeImageMsg(*new_pts_, 1);
                im_pub_.publish(image_ptr);
            }
          }
          
          {
            // ros::WallTime// start = ros::WallTime::now();
            
            //Lock mutex
            ReadLock lock(config_mutex_);
            
            std::swap(next_pts_,old_pts_);
            if(!next_pts_)
            {
              next_pts_ = utils::getECWrapper(config_.height, config_.width,config_.vfov,true);
            }
            else
            {
              next_pts_->init(config_.height, config_.width, config_.vfov, true);
            }
            
            // ROS_DEBUG_STREAM_NAMED("timing", "Reinitting data structure took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          }
          
        
        }
        
        std::swap(new_pts_, old_pts_);  
        
        // ROS_DEBUG_STREAM_NAMED("timing", "Total time: " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
    }
    
    void DedicatedEgoCylindricalPropagator::connectCB()
    {
        // If no one is listening, we can propagate points in place
        if (im_pub_->get_subscription_count() == 0)
        {
            
        }
    }
    
    void DedicatedEgoCylindricalPropagator::configCB(const egocylindrical::PropagatorConfig &config, uint32_t level)
    {
        WriteLock lock(config_mutex_);
     
        // ROS_INFO_STREAM("Updating propagator config: height=" << config.height << ", width=" << config.width << ", vfov=" << config.vfov);
        config_ = config;
    }
     
    bool DedicatedEgoCylindricalPropagator::init()
    {
        reconfigure_server_->setCallback(std::bind(&DedicatedEgoCylindricalPropagator::configCB, this, _1, _2));
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/2;        
        
        cylinder_width_ = 2048;
        cylinder_height_ = 320;
        
        transformed_pts_ = utils::getECWrapper(config_.height, config_.width,config_.vfov,true);
        
        next_pts_ = utils::getECWrapper(config_.height, config_.width,config_.vfov, true);
                
        
        // Get topic names
        std::string depth_topic="/camera/depth/image_raw", info_topic= "/camera/depth/camera_info", points_topic="image_out", filtered_pc_topic="filtered_points";
        
        pnh_.getParam("image_in", depth_topic );
        pnh_.getParam("info_in", info_topic );
        pnh_.getParam("image_out", points_topic );
        pnh_.getParam("filtered_points", filtered_pc_topic);
        
        // Setup publishers
        ros::SubscriberStatusCallback image_cb = std::bind(&DedicatedEgoCylindricalPropagator::connectCB, this);        
        im_pub_ = nh_.advertise<sensor_msgs::msg::Image>(points_topic, 1, image_cb, image_cb);
        
        //ros::SubscriberStatusCallback pc_cb = std::bind(&DedicatedEgoCylindricalPropagator::connectCB, this);        
        pc_pub_ = nh_.advertise<sensor_msgs::msg::PointCloud2>(filtered_pc_topic, 3);
        
        
        // Setup subscribers
        depthSub.subscribe(it_, depth_topic, 3);
        depthInfoSub.subscribe(nh_, info_topic, 3);
        
        // Ensure that CameraInfo is transformable
        info_tf_filter = std::make_shared<tf_filter>(depthInfoSub, buffer_, "odom", 2,nh_);
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer = std::make_shared<synchronizer>(depthSub, *info_tf_filter, 2);
        timeSynchronizer->registerCallback(std::bind(&DedicatedEgoCylindricalPropagator::update, this, _1, _2));
        
        return true;
    }

    DedicatedEgoCylindricalPropagator::DedicatedEgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh),
        tf_listener_(buffer_),
        it_(nh)
    {
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        
        
    }
    
    DedicatedEgoCylindricalPropagator::~DedicatedEgoCylindricalPropagator()
    {
      
    }
      


}

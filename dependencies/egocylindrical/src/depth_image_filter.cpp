

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>

// #include <dynamic_reconfigure/server.h>
#include <egocylindrical/FilterConfig.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <boost/thread/mutex.hpp>

namespace egocylindrical
{

  class DepthImageFilter
  {
    
  private:
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    
    image_transport::SubscriberFilter im_sub_;
    ros::Publisher im_pub_;
    
    boost::mutex mutex_;
    typedef boost::mutex::scoped_lock Lock;
    
    
    typedef egocylindrical::FilterConfig ConfigType;
    ConfigType config_;
    
    typedef dynamic_reconfigure::Server<ConfigType> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> reconfigure_server_;
    
    
    
  public:
    DepthImageFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Image filter Initialized"<<std::endl;
    }
    
    bool init()
    {

      
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        reconfigure_server_->setCallback(std::bind(&DepthImageFilter::configCB, this, _1, _2));
      
        ros::SubscriberStatusCallback info_cb = std::bind(&DepthImageFilter::ssCB, this);
        im_pub_ = nh_.advertise<sensor_msgs::msg::Image>("image_out", 2, info_cb, info_cb);
        
        im_sub_.registerCallback(std::bind(&DepthImageFilter::imageCB, this, _1));

        return true;
    }

  private:
    
    void ssCB()
    {
        
      if(im_pub_->get_subscription_count()>0)
        {
            //Note: should probably add separate checks for each
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                im_sub_.subscribe(it_, "image_in", 2);
                // ROS_INFO("Depth Image Filter Subscribing");
            }
      
        }
        else
        {
            im_sub_.unsubscribe();
            // ROS_INFO("RangeImage Converter Unsubscribing");
        }
    }

    //NOTE: Once the parameters have been moved to their own message, this should subscribe to the parameters instead
    void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
    {
        // ROS_INFO("Received image msg");
        
        // This may be redundant now
        if(im_pub_->get_subscription_count() > 0)
        {
          try
          {
            // ros::WallTime// start = ros::WallTime::now();
            cv_bridge::CvImage::ConstPtr cv_in = cv_bridge::toCvCopy(image_msg);
            
            //cv_bridge::CvImage cv_out;
            //cv_out.header = image_msg->header;
            //cv_out.encoding = image_msg->encoding;
            //cv_out.image = cv_bridge::toCvCopy(image_msg);
            
            cv_bridge::CvImage::ConstPtr cv_out = cv_bridge::toCvCopy(image_msg);
            
            cv::setNumThreads(0);
            
            {
              Lock(mutex_);
              cv::bilateralFilter(cv_in->image, cv_out->image, config_.diameter, config_.sigmaColor, config_.sigmaSpace);
            }

            // ROS_DEBUG_STREAM("Filtering depth image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                      
            im_pub_.publish(cv_out->toImageMsg());
            
          }
          catch (cv_bridge::Exception& e)
          {
            // ROS_ERROR(("cv_bridge exception: %s", e.what());
            return;
          }
        }
        
    }
    
    void configCB(const ConfigType &config, uint32_t level)
    {
      //Atomic operation, so no need for mutex this time
      //WriteLock lock(config_mutex_);
      
      // ROS_INFO_STREAM("Updating Depth Image Filter config:");
      {
        Lock(mutex_);
        config_ = config;
      }
    }

  };

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_filter_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  egocylindrical::DepthImageFilter s(nh, pnh);
  s.init();
  ros::spin();
}

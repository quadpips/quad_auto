//
// Created by root on 2/5/18.
//

#include <egocylindrical/can_image_inflator_generator.h>
//#include <egocylindrical/range_image_inflator_core.h>
#include <algorithm>

// The below are redundant
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <egocylindrical/ecwrapper.h>
#include <sensor_msgs/image_encodings.hpp>

namespace egocylindrical
{



    CanImageInflatorGenerator::CanImageInflatorGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Can Image Node Initialized"<<std::endl;
    }
    
    bool CanImageInflatorGenerator::init()
    {
        //use_raw_ = false;
        std::string image_topic = "inflated_image";
                
        pnh_.getParam("image_topic", image_topic );
        
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        reconfigure_server_->setCallback(std::bind(&CanImageInflatorGenerator::configCB, this, _1, _2));

        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer_ = std::make_shared<synchronizer>(im_sub_, ec_sub_, 2);
        timeSynchronizer_->registerCallback(std::bind(&CanImageInflatorGenerator::imgCB, this, _1, _2));
        
        image_transport::SubscriberStatusCallback image_cb = std::bind(&CanImageInflatorGenerator::ssCB, this);
        {
            Lock lock(connect_mutex_);
            im_pub_ = it_.advertise(image_topic, 2, image_cb, image_cb);
        }
        return true;
    }
    
    void CanImageInflatorGenerator::configCB(const ConfigType &config, uint32_t level)
    {
        Lock lock(config_mutex_);
        
        // ROS_INFO_STREAM("Updating Can Image Inflator config: num_threads=" << config.num_threads << ", inflation_radius=" << config.inflation_radius);
        config_ = config;
    }
    
    void CanImageInflatorGenerator::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_->get_subscription_count() << std::endl;
        Lock lock(connect_mutex_);
        if(im_pub_->get_subscription_count()>0)
        {
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {
              
            }
            else
            {
                im_sub_.subscribe(it_, "can_image", 2);
                ec_sub_.subscribe(nh_, "egocylindrical_points", 2);
                
                // ROS_INFO("RangeImage Inflator Subscribing");
            }  
        }
        else
        {
            im_sub_.unsubscribe();
            ec_sub_.unsubscribe();
            
            // ROS_INFO("RangeImage Inflator Unsubscribing");
        }
    }
    
    bool isknown(uint16_t range)
    {
      return range>0;
    }
    
    bool isknown(float range)
    {
      return range==range;
    }
    
    template<typename T>
    float getInflationAngle(T range, T inflation_radius)
    {
      return std::asin(float(inflation_radius)/range);
    }
    
    template<typename T>
    int getNumInflationIndices(T range, float scale, T inflation_radius)
    {
      return inflation_radius * scale / range;
    }
    
    template<typename T>
    void inflateRowRegion(T range, int start_ind, int end_ind, T* inflated)
    {
      for(int ind = start_ind; ind < end_ind; ind++)
      {
        T& val = inflated[ind];
        
        if(!isknown(val) || range < val)
        {
          val = range;
        }
        
//         if(inflated[ind]< range)
//         {
//         }
//         else
//         {
//           inflated[ind] = range;
//         }
      }
    }
    

    
//     float convertRange(float range)
//     {
//       return range;
//     }
//     
//     float convertRange(uint16_t range)
//     {
//       return float(range)/1000;
//     }
    
    void convertRange(float in, float& out)
    {
      out = in;
    }
    
    void convertRange(float in, uint16_t& out)
    {
      out = 1000*in;
    }
    
    
    
    template<typename T>
    void inflateRow(const T* ranges, int width, float scale, T inflation_radius, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        T range = ranges[i];

        if(!isknown(range))
        {
          continue;
        }
        
        int inflation_size = getNumInflationIndices(range, scale, inflation_radius);
        
        int start_ind = i - inflation_size;
        int end_ind = i + inflation_size + 1;
        
        start_ind = std::max(0, start_ind);
        end_ind = std::min(width, end_ind);
        //if(range >  inflation_radius)
        {
          //T modified_range = range - inflation_radius;
          T modified_range = range;

          inflateRowRegion(modified_range, start_ind, end_ind, inflated);
        }
      }
    }
    
    template<typename T>
    void inflateHorizontally(const T* ranges, int height, int width, float scale, T inflation_radius, int num_threads, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        inflateRow(ranges+j*width, width, scale, inflation_radius, inflated+j*width);
      }
    }
    
    template<typename T>
    void inflateColumn(int height, int width, float scale, T inflation_radius, T* buffer, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        T range = buffer[j*width];
        
        if(!isknown(range))
        {
          continue;
        }
        int inflation_size = getNumInflationIndices(range, scale, inflation_radius);  //Need to think this through, might not be same equation
        
        int start_ind = std::max(j - inflation_size, 0);
        int end_ind = std::min(j + inflation_size + 1, height);
                
        for(int k = start_ind; k < end_ind; k++)
        {
          T& val = inflated[k*width];
          if(!isknown(val) || range < val)
          {
            val = range;
          }
        }
      }
    }
    
    template<typename T>
    void inflateVertically(int height, int width, float scale, T inflation_radius, int num_threads, T* buffer, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        inflateColumn(height, width, scale, inflation_radius, buffer+i, inflated+i);
      }
    }
    
    template<typename T>
    void inflateCanImage(const T* ranges, const utils::ECConverter& converter, T inflation_radius, int num_threads, T* buffer, T* inflated)
    {
      int width = converter.getCanWidth();
      float scale = converter.getCanScale();
      
      inflateHorizontally(ranges, 2*width, width, scale, inflation_radius, num_threads, buffer);
      inflateVertically(width, width, scale, inflation_radius, num_threads, buffer, inflated);
      inflateVertically(width, width, scale, inflation_radius, num_threads, buffer+width*width, inflated+width*width);
    }
    
    template<typename T>
    void inflateCanImage(const sensor_msgs::msg::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, int num_threads, sensor_msgs::msg::Image& new_msg, const T unknown_value)
    {
      T converted_inflation_radius;
      convertRange(inflation_radius, converted_inflation_radius);
      int width = converter.getCanWidth();
      int num_pnts = width*width*2;
      
      T* inflated_ranges = (T*)new_msg.data.data();
      std::fill(inflated_ranges, inflated_ranges+num_pnts, unknown_value);
      
      std::vector<T> buffer(num_pnts, unknown_value);
      
      
      const T* ranges = (T*)range_msg.data.data();
      
      inflateCanImage<T>(ranges, converter, converted_inflation_radius, num_threads, buffer.data(), inflated_ranges);
    }
    
    
    sensor_msgs::msg::Image::ConstSharedPtr getInflatedCanImageMsg(const EgoCylinderPoints::ConstPtr& ec_msg, const sensor_msgs::msg::Image::ConstSharedPtr& range_msg, float inflation_radius, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg)
    {
      sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
      
      sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
      new_msg.header = range_msg->header;
      new_msg.height = range_msg->height;
      new_msg.width = range_msg->width;
      new_msg.encoding = range_msg->encoding;
      new_msg.is_bigendian = range_msg->is_bigendian;
      new_msg.step = range_msg->step;
      
      size_t size = new_msg.step * new_msg.height;
      
      //// ros::WallTime// start = ros::WallTime::now();
      
      new_msg.data.resize(size);
      //cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::TYPE_32FC1, new_im_).toImageMsg();
      
      
      // // ROS_INFO_STREAM_NAMED("timing","Allocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
      
      utils::ECConverter converter;
      converter.fromCameraInfo(ec_msg);
      
//       float* inflated_ranges = (float*)new_msg.data.data();
//       std::fill(inflated_ranges, inflated_ranges+converter.getCols(), utils::dNaN);
//       
//       const float* ranges = (float*)range_msg->data.data();
      
      if(range_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        inflateCanImage<float>(*range_msg, converter, inflation_radius, num_threads, new_msg, utils::dNaN);
      }
      else if(range_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        inflateCanImage<uint16_t>(*range_msg, converter, inflation_radius, num_threads, new_msg, 0);
      }
      else
      {
        // // ROS_ERROR(_STREAM("Unsupported image format for inflation: " << range_msg->encoding);
      }
      
      
      
      return new_msg_ptr;
    }

    
    void CanImageInflatorGenerator::imgCB(const sensor_msgs::msg::Image::ConstSharedPtr& range_msg, const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg)
    {
        // ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // This may be redundant now
        if(im_pub_->get_subscription_count() > 0)
        {

          // ros::WallTime// start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
                
          ConfigType config;
          {
            Lock lock(config_mutex_);
            config = config_;
          }
          sensor_msgs::msg::Image::ConstSharedPtr image_ptr = getInflatedCanImageMsg(ec_msg, range_msg, config.inflation_radius, config.num_threads, preallocated_msg_);

          // ROS_DEBUG_STREAM_NAMED("timing","Inflating egocan lid image by {" << config.inflation_radius << "x" << config.inflation_height/2 << "} took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          // ROS_DEBUG("publish egocylindrical image");
          
          im_pub_.publish(image_ptr);
          
          // start = ros::WallTime::now();
          preallocated_msg_= std::make_shared<sensor_msgs::msg::Image>();
          preallocated_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          // ROS_DEBUG_STREAM_NAMED("timing","Preallocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          
        }
        
    }




}

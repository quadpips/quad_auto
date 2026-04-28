//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_inflator_generator.h>
//#include <egocylindrical/range_image_inflator_core.h>
#include <algorithm>

// The below are redundant
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <egocylindrical/ecwrapper.h>
#include <sensor_msgs/image_encodings.hpp>
// #include <benchmarking_tools/benchmarking_tools.h>

namespace egocylindrical
{



    RangeImageInflatorGenerator::RangeImageInflatorGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Range Image Node Initialized"<<std::endl;

        
        
    }
    
    bool RangeImageInflatorGenerator::init()
    {
        //use_raw_ = false;
        std::string image_topic = "inflated_image";
                
        pnh_.getParam("image_topic", image_topic );
        
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        reconfigure_server_->setCallback(std::bind(&RangeImageInflatorGenerator::configCB, this, _1, _2));
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer_ = std::make_shared<synchronizer>(im_sub_, ec_sub_, 2);
        timeSynchronizer_->registerCallback(std::bind(&RangeImageInflatorGenerator::imgCB, this, _1, _2));
        
        image_transport::SubscriberStatusCallback image_cb = std::bind(&RangeImageInflatorGenerator::ssCB, this);
        {
            Lock lock(connect_mutex_);
            im_pub_ = it_.advertise(image_topic, 2, image_cb, image_cb);
        }
        return true;
    }
    
    void RangeImageInflatorGenerator::configCB(const ConfigType &config, uint32_t level)
    {
      Lock lock(config_mutex_);
      
      // ROS_INFO_STREAM("Updating Range Image Inflator config: num_threads=" << config.num_threads << ", inflation_radius=" << config.inflation_radius);
      config_ = config;
    }
    
    void RangeImageInflatorGenerator::ssCB()
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
                im_sub_.subscribe(it_, "range_image", 2);
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
    int getNumRowInflationIndices(T range, float scale, T inflation_radius)  //TODO: construct this using existing functions in ECConverter
    {
      return getInflationAngle(range, inflation_radius) * scale;
    }
    
    template<typename T>
    int getNumColInflationIndices(T range, float scale, T inflation_height)
    {
      return inflation_height * scale / range;
    }
    
    
    template<typename T>
    void inflateRowRegion(T range, int start_ind, int end_ind, T* inflated)
    {
      #pragma GCC ivdep
      for(int ind = start_ind; ind < end_ind; ind++)
      {
        T val = inflated[ind];
        
        //inflated[ind] = (!isknown(val) || range < val) ? range : val;
        
        //T mod_val = isknown(val) ? val : std::numeric_limits<T>::max();
        T mod_val = (val==val) ? val : std::numeric_limits<T>::max();
        inflated[ind] = (range < mod_val) ? range : mod_val;
                
//         if(!isknown(val) || range < val)
//         {
//           val = range;
//         }
        
        
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
        
        int inflation_size = getNumRowInflationIndices(range, scale, inflation_radius);
        
        int start_ind = i - inflation_size;
        int end_ind = i + inflation_size + 1;
        
        if(range >  inflation_radius)
        {
          T modified_range = range; // - inflation_radius;
          
          if(start_ind < 0)
          {
            inflateRowRegion(modified_range, start_ind+width, width, inflated);
            inflateRowRegion(modified_range, 0, end_ind, inflated);
          }
          else if(end_ind >= width)
          {
            inflateRowRegion(modified_range, start_ind, width, inflated);
            inflateRowRegion(modified_range, 0, end_ind-width, inflated);
          }
          else
          {
            inflateRowRegion(modified_range, start_ind, end_ind, inflated);
          }
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
    void getColInflationIndices(int height, float scale, T inflation_radius, T inflation_height, bool conservative, int row, T range, int& start_ind, int& end_ind)
    {
      T row_factor = (row - height/2) * range;
      T height_factor = inflation_height * scale;
      
      T top_front_ind = (row_factor - height_factor)/(range + inflation_radius);
      T top_back_ind = (row_factor - height_factor)/(range - inflation_radius);
      T bottom_front_ind = (row_factor + height_factor)/(range + inflation_radius);
      T bottom_back_ind = (row_factor + height_factor)/(range - inflation_radius);
      
      //int top_ind = (row_factor < height_factor) ? top_front_ind : top_back_ind;
      
      start_ind = std::min(top_front_ind, top_back_ind) + height/2;
      end_ind = std::max(bottom_front_ind, bottom_back_ind) + height/2;
    }
    
    
    template<typename T>
    void inflateColumn(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, T* buffer, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        T range = buffer[j*width];
        
        if(!isknown(range))
        {
          continue;
        }
        
        bool use_new = false;

        int inflation_size = getNumColInflationIndices(range, scale, inflation_height);  //Need to think this through, might not be same equation
        int old_start_ind = std::max(j - inflation_size, 0);
        int old_end_ind = std::min(j + inflation_size + 1, height);
        
        int raw_start_ind, raw_end_ind;
//         getColInflationIndices(height, scale, inflation_radius, inflation_height, conservative, j, range, raw_start_ind, raw_end_ind);
        
        int start_ind, end_ind;
        if(use_new)
        {
          start_ind = std::max(raw_start_ind, 0);
          end_ind = std::min(raw_end_ind+1, height);
        }
        else
        {
          start_ind = old_start_ind;
          end_ind = old_end_ind;
        }
        
        //ROS_DEBUG_STREAM_THROTTLE_NAMED("vertical_inflation.index_changes", "Old start=" << old_start_ind << ", new start=" << start_ind << ", old end=" << old_end_ind << ", new end=" << end_ind);
        if(old_start_ind != start_ind || old_end_ind != end_ind)
        {
          //ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "vertical_inflation.index_changes", "Old start=" << old_start_ind << ", new start=" << start_ind << ", old end=" << old_end_ind << ", new end=" << end_ind);
        }
        
        if(range >  inflation_radius)
        {
          T modified_range = range - inflation_radius;
        
          for(int k = start_ind; k < end_ind; k++)
          {
            T& val = inflated[k*width];
            if(!isknown(val) || modified_range < val)
            {
              val = modified_range;
            }
          }
        }
      }
    }
    
    template<typename T>
    void inflateVertically(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        inflateColumn(height, width, scale, inflation_radius, inflation_height, conservative, buffer+i, inflated+i);
      }
    }
    
    template<typename T>
    void inflateRangeImage(const T* ranges, const utils::ECConverter& converter, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
    {
      int height = converter.getHeight();
      int width = converter.getWidth();
      
      float hscale = converter.getHScale();
      float vscale = converter.getVScale();
      
      inflateHorizontally(ranges, height, width, hscale, inflation_radius, num_threads, buffer);
      inflateVertically(height, width, vscale, inflation_radius, inflation_height, conservative, num_threads, buffer, inflated);
    }
    
    template<typename T>
    void inflateRangeImage(const sensor_msgs::msg::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, bool conservative, int num_threads, sensor_msgs::msg::Image& new_msg, const T unknown_value)
    {
      T converted_inflation_radius;
      convertRange(inflation_radius, converted_inflation_radius);
      
      T converted_inflation_height;
      convertRange(inflation_height, converted_inflation_height);
      
      std::vector<T> buffer(converter.getCols(), unknown_value);
      
      T* inflated_ranges = (T*)new_msg.data.data();
      std::fill(inflated_ranges, inflated_ranges+converter.getCols(), unknown_value);
      
      const T* ranges = (T*)range_msg.data.data();
      
      inflateRangeImage<T>(ranges, converter, converted_inflation_radius, converted_inflation_height, conservative, num_threads, buffer.data(), inflated_ranges);
    }
    
    
    sensor_msgs::msg::Image::ConstSharedPtr getInflatedRangeImageMsg(const EgoCylinderPoints::ConstPtr& ec_msg, const sensor_msgs::msg::Image::ConstSharedPtr& range_msg, float inflation_radius, float inflation_height, bool conservative, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg)
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
        inflateRangeImage<float>(*range_msg, converter, inflation_radius, inflation_height, conservative, num_threads, new_msg, utils::dNaN);
      }
      else if(range_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        inflateRangeImage<uint16_t>(*range_msg, converter, inflation_radius, inflation_height, conservative, num_threads, new_msg, 0);
      }
      else
      {
        // // ROS_ERROR(_STREAM("Unsupported image format for inflation: " << range_msg->encoding);
      }
      
      
      
      return new_msg_ptr;
    }

    
    void RangeImageInflatorGenerator::imgCB(const sensor_msgs::msg::Image::ConstSharedPtr& range_msg, const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg)
    {
        // ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // This may be redundant now
        if(im_pub_->get_subscription_count() > 0)
        {
          DURATION_DEBUG_STREAM_THROTTLED("inflation", 100, 1);
          // ros::WallTime// start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
          bool conservative = false;
          
          ConfigType config;
          {
            Lock lock(config_mutex_);
            config = config_;
          }
          sensor_msgs::msg::Image::ConstSharedPtr image_ptr = getInflatedRangeImageMsg(ec_msg, range_msg, config.inflation_radius, config.inflation_height/2, conservative, config.num_threads, preallocated_msg_);

          // ROS_DEBUG_STREAM_NAMED("timing","Inflating range image by {" << config.inflation_radius << "x" << config.inflation_height/2 << "} took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          // ROS_DEBUG("publish egocylindrical image");
          
          im_pub_.publish(image_ptr);
          
          // start = ros::WallTime::now();
          preallocated_msg_= std::make_shared<sensor_msgs::msg::Image>();
          preallocated_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          // ROS_DEBUG_STREAM_NAMED("timing","Preallocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          
        }
        
    }




}

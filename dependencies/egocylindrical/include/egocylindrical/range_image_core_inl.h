#ifndef RANGE_IMAGE_CORE_INL_H
#define RANGE_IMAGE_CORE_INL_H

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/to_ieee754.h>

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <stdint.h>


namespace egocylindrical
{
    
    namespace utils
    {
        template <typename T>
        bool isNan(T val)
        {
          return toIEEE754(val).isNan();
        }
        
//         template <>
//         bool isNan<uint16_t>(uint16_t val)
//         {
//           return false;
//         }
        
        template <typename T,uint scale>
        void generateRangeImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const float* const x = cylindrical_history.getX();
            const float* const z = cylindrical_history.getZ();

            int num_cols = cylindrical_history.getCols();

            #pragma GCC ivdep
            for(int j = 0; j < num_cols; ++j)
            {
                T temp = unknown_val;
                
                if(x[j]==x[j])
                {
                    /* NOTE: It appears that the conversion from float to uint16 is what is preventing this from vectorizing for the uint16 case,
                     * which actually makes sense: the datatypes have different widths, so how could the conversions be vectorized?
                     * It should be possible to vectorize the calculation and have the conversion separate, though unclear whether that would be faster.
                     */
                    temp = std::sqrt(x[j]*x[j] + z[j]*z[j]) * scale;                    
                }

                r[j] = temp;
            }
        }
        
        
        template <typename T, uint scale>
        sensor_msgs::msg::Image::SharedPtr generateRangeImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, const T unknown_val, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = cylindrical_history.getHeight();
            new_msg.width = cylindrical_history.getWidth();
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false; //image->is_bigendian;
            new_msg.step = cylindrical_history.getWidth() * sizeof(T); //sensor_msgs::image_encodings::bitDepth(encoding); // cylindrical_history.elemSize(); // Ideally, replace this with some other way of getting size
            size_t size = new_msg.step * cylindrical_history.getHeight();
            
            //// ros::WallTime// start = ros::WallTime::now();
            
            new_msg.data.resize(size);
            //cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::TYPE_32FC1, new_im_).toImageMsg();
            
            
           // // ROS_INFO_STREAM_NAMED("timing","Allocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
            
            T* data = (T*)new_msg.data.data();
            
            generateRangeImage<T, scale>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg);

        
        template <> 
        sensor_msgs::msg::Image::SharedPtr generateRangeImageMsg<float>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            return generateRangeImageMsg<float, 1>(cylindrical_history, sensor_msgs::image_encodings::TYPE_32FC1, dNaN, num_threads, preallocated_msg);
        }
        
        template <>
        sensor_msgs::msg::Image::SharedPtr generateRangeImageMsg<uint16_t>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            return generateRangeImageMsg<uint16_t, 1000>(cylindrical_history, sensor_msgs::image_encodings::TYPE_16UC1, 0, num_threads, preallocated_msg);
        }

        
    }
}

#endif //RANGE_IMAGE_CORE_INL_H

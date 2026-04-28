#ifndef EGOCYLINDRICAL_CAN_DEPTH_IMAGE_CORE_INL_H
#define EGOCYLINDRICAL_CAN_DEPTH_IMAGE_CORE_INL_H

#include <egocylindrical/ecwrapper.h>

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <stdint.h>
#include <cmath>

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
        void generateCanImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const float* const y = cylindrical_history.getY();

            int num_cols = cylindrical_history.getCols();
            int num_pnts = cylindrical_history.getNumPts();

            #pragma GCC ivdep
            for(int j = num_cols; j < num_pnts; ++j)
            {
                T temp = unknown_val;
                
                float depth = std::abs(y[j]);
                
                if(depth==depth)
                {
                    /* NOTE: It appears that the conversion from float to uint16 is what is preventing this from vectorizing for the uint16 case,
                     * which actually makes sense: the datatypes have different widths, so how could the conversions be vectorized?
                     * It should be possible to vectorize the calculation and have the conversion separate, though unclear whether that would be faster.
                     */
                    temp = depth * scale;                    
                }

                r[j-num_cols] = temp;
            }
        }
        
        
        template <typename T, uint scale>
        sensor_msgs::msg::Image::SharedPtr generateCanImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                  const T unknown_val, int num_threads, 
                                                  sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * sizeof(T);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            T* data = (T*)new_msg.data.data();
            
            generateCanImage<T, scale>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg);

        
        template <> 
        sensor_msgs::msg::Image::SharedPtr generateCanImageMsg<float>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateCanImageMsg<float, 1>(cylindrical_history, sensor_msgs::image_encodings::TYPE_32FC1, 
                                                dNaN, num_threads, preallocated_msg);
        }
        
        template <>
        sensor_msgs::msg::Image::SharedPtr generateCanImageMsg<uint16_t>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateCanImageMsg<uint16_t, 1000>(cylindrical_history, sensor_msgs::image_encodings::TYPE_16UC1, 
                                                      0, num_threads, preallocated_msg);
        }

        
    }
}

#endif //EGOCYLINDRICAL_CAN_DEPTH_IMAGE_CORE_INL_H

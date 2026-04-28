#ifndef EGOCYLINDRICAL_FLOOR_DEPTH_IMAGE_CORE_INL_H
#define EGOCYLINDRICAL_FLOOR_DEPTH_IMAGE_CORE_INL_H

#include <egocylindrical/ecwrapper.h>

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

        /////////////////////////
        // COLORED LABEL IMAGE //
        /////////////////////////
        void generateFloorNormalColoredImage(const utils::ECWrapper& cylindrical_history, uint32_t* data, const uint32_t unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const float* const norm_x = cylindrical_history.getNormalX();
            const float* const norm_y = cylindrical_history.getNormalY();
            const float* const norm_z = cylindrical_history.getNormalZ();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                int j_ = j - start_idx;
                std::uint8_t r = 0, g = 0, b = 0, a = 0;    // Example: Red color
                cv::Point3f normal(norm_x[j], norm_y[j], norm_z[j]);

                a = (std::sqrt( normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) > 0 ? 255 : 0);

                r = uint8_t(std::abs(normal.x) * 255);
                g = uint8_t(std::abs(normal.y) * 255);
                b = uint8_t(std::abs(normal.z) * 255);

                std::uint32_t rgba = ((std::uint32_t)a << 24 | (std::uint32_t)b << 16 | (std::uint32_t)g << 8 | (std::uint32_t)r);

                data[j_] = rgba;
            }
        }
        
        sensor_msgs::msg::Image::SharedPtr generateFloorNormalColoredImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                                const uint32_t unknown_val, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * 4 * sizeof(uint8_t);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            uint32_t * data = (uint32_t*)new_msg.data.data();
            
            generateFloorNormalColoredImage(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        sensor_msgs::msg::Image::SharedPtr generateFloorNormalColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, 
                                                                sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateFloorNormalColoredImageMsg(cylindrical_history, sensor_msgs::image_encodings::RGBA8, 
                                                    0, num_threads, preallocated_msg);
        }

        ///////////////////
        // NORMALS IMAGE //
        ///////////////////
        template <typename T>
        void generateFloorNormalImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const float* const norm_x = cylindrical_history.getNormalX();
            const float* const norm_y = cylindrical_history.getNormalY();
            const float* const norm_z = cylindrical_history.getNormalZ();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                int j_ = j - start_idx;
                r[3 * j_ + 0] = norm_x[j];
                r[3 * j_ + 1] = norm_y[j];
                r[3 * j_ + 2] = norm_z[j];
            }
        }
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateFloorNormalImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                            const T unknown_val, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * 3 * sizeof(T);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            T* data = (T*)new_msg.data.data();
            
            generateFloorNormalImage<T>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateFloorNormalImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg);

        template <>
        sensor_msgs::msg::Image::SharedPtr generateFloorNormalImageMsg<float>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateFloorNormalImageMsg<float>(cylindrical_history, sensor_msgs::image_encodings::TYPE_32FC3, 
                                                      0, num_threads, preallocated_msg);
        }

        /////////////////////////
        // COLORED LABEL IMAGE //
        /////////////////////////
        void generateFloorLabelColoredImage(const utils::ECWrapper& cylindrical_history, uint32_t* data, const uint32_t unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const uint8_t* const labels = cylindrical_history.getLabels();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                int j_ = j - start_idx;
                std::uint8_t r = 0, g = 0, b = 0, a = 0;    // Example: Red color

                if (labels[j] == 0) // VOID
                {
                    // // ROS_DEBUG_STREAM_NAMED("labels", " label is void");
                    r = 0; g = 0; b = 0; a = 0;
                } else if (labels[j] == 1) // NONPASS
                {
                    // // ROS_DEBUG_STREAM_NAMED("labels", " label is non-passable");
                    r = 255; g = 0; b = 0; a = 255;
                }  else if (labels[j] == 2) // PASS
                {
                    // // ROS_DEBUG_STREAM_NAMED("labels", " label is passable");
                    r = 255; g = 255; b = 0; a = 255;
                }  else if (labels[j] == 3) // STEP
                {
                    // // ROS_DEBUG_STREAM_NAMED("labels", " label is steppable");
                    r = 0; g = 255; b = 0; a = 255;
                } else
                {
                    // // ROS_DEBUG_STREAM_NAMED("labels", " label not recognized!");
                }

                std::uint32_t rgba = ((std::uint32_t)a << 24 | (std::uint32_t)b << 16 | (std::uint32_t)g << 8 | (std::uint32_t)r);

                data[j_] = rgba;
            }
        }
        
        sensor_msgs::msg::Image::SharedPtr generateFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                                const uint32_t unknown_val, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * 4 * sizeof(uint8_t);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            uint32_t * data = (uint32_t*)new_msg.data.data();
            
            generateFloorLabelColoredImage(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        sensor_msgs::msg::Image::SharedPtr generateFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, 
                                                                sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateFloorLabelColoredImageMsg(cylindrical_history, sensor_msgs::image_encodings::RGBA8, 
                                                    0, num_threads, preallocated_msg);
        }

        /////////////////
        // LABEL IMAGE //
        /////////////////
        template <typename T>
        void generateFloorLabelImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const uint8_t* const labels = cylindrical_history.getLabels();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                int j_ = j - start_idx;
                r[j_] = labels[j];
            }
        }
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                            const T unknown_val, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * sizeof(T);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            T* data = (T*)new_msg.data.data();
            
            generateFloorLabelImage<T>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg);

        template <>
        sensor_msgs::msg::Image::SharedPtr generateFloorLabelImageMsg<uint8_t>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateFloorLabelImageMsg<uint8_t>(cylindrical_history, sensor_msgs::image_encodings::TYPE_8UC1, 
                                                      0, num_threads, preallocated_msg);
        }

        /////////////////
        // RANGE IMAGE //
        /////////////////

        template <typename T,uint scale>
        void generateFloorImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            // // ROS_DEBUG("Generating image of cylindrical memory");
            
            const float* const y = cylindrical_history.getY();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
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

                int j_ = j - start_idx;
                r[j_] = temp;
            }
        }
        
        
        template <typename T, uint scale>
        sensor_msgs::msg::Image::SharedPtr generateFloorImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                  const T unknown_val, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::msg::Image::SharedPtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : std::make_shared<sensor_msgs::msg::Image>();
            
            sensor_msgs::msg::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * sizeof(T);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            T* data = (T*)new_msg.data.data();
            
            generateFloorImage<T, scale>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::msg::Image::SharedPtr generateFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg);

        
        template <> 
        sensor_msgs::msg::Image::SharedPtr generateFloorImageMsg<float>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateFloorImageMsg<float, 1>(cylindrical_history, sensor_msgs::image_encodings::TYPE_32FC1, 
                                                dNaN, num_threads, preallocated_msg);
        }
        
        template <>
        sensor_msgs::msg::Image::SharedPtr generateFloorImageMsg<uint16_t>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr& preallocated_msg)
        {
          return generateFloorImageMsg<uint16_t, 1000>(cylindrical_history, sensor_msgs::image_encodings::TYPE_16UC1, 
                                                      0, num_threads, preallocated_msg);
        }

        
    }
}

#endif //EGOCYLINDRICAL_FLOOR_DEPTH_IMAGE_CORE_INL_H

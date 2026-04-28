#include <egocylindrical/ecwrapper.h>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>

#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace egocylindrical
{
    
    namespace utils
    {
        /* TODO: ensure that the right size 'ints' are used everywhere. On my current system, the size of int = minimum size of long int, so 32 bit system might fail */
        sensor_msgs::msg::PointCloud2::ConstSharedPtr generate_normal_point_cloud(const utils::ECWrapper& points)
        {
            const int num_pts = points.getNumPts();
          
            const int num_cols = points.getCols();
            
            pcl::PointCloud<pcl::PointXYZRGBA> pcloud;
            
            sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            pcl::toROSMsg(pcloud, *pcloud_msg);
            
            // // ROS_DEBUG_STREAM_NAMED("labels", "sizeof(pcl::PointXYZ): " << sizeof(pcl::PointXYZ));
            // // ROS_DEBUG_STREAM_NAMED("labels", "sizeof(pcl::PointXYZRGB): " << sizeof(pcl::PointXYZRGB));
            // // ROS_DEBUG_STREAM_NAMED("labels", "sizeof(pcl::PointXYZI): " << sizeof(pcl::PointXYZI));
            pcloud_msg->data.resize(sizeof(pcl::PointXYZRGBA) * num_pts);
            
            pcloud_msg->width = num_pts;
            pcloud_msg->height = 1;
            
            const float* __restrict__ x = points.getX();
            const float* __restrict__ y = points.getY();
            const float* __restrict__ z = points.getZ();
            const float* __restrict__ norm_x = points.getNormalX();
            const float* __restrict__ norm_y = points.getNormalY();
            const float* __restrict__ norm_z = points.getNormalZ();            

            float* data = (float*) pcloud_msg->data.data();

            std::uint8_t r = 0, g = 0, b = 0, a = 0;    // Example: Red color

            //#pragma omp parallel for num_threads(4)
            #pragma GCC ivdep
            for(int j = 0; j < num_cols; ++j) //Vectorization requires -fno-trapping-math -fno-math-errno
            {   
                cv::Point3f point(x[j],y[j],z[j]);
                cv::Point3f Pcyl_t = points.projectWorldToCylinder(point);

                cv::Point3f normal(norm_x[j], norm_y[j], norm_z[j]);

                a = (std::sqrt( normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) > 0 ? 255 : 0);

                r = uint8_t(std::abs(normal.x) * 255);
                g = uint8_t(std::abs(normal.y) * 255);
                b = uint8_t(std::abs(normal.z) * 255);

                std::uint32_t rgba = ((std::uint32_t)a << 24 | (std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
                float color_f = *reinterpret_cast<float*>(&rgba);

                data[8*j] =   Pcyl_t.x;
                data[8*j+1] = Pcyl_t.y;
                data[8*j+2] = Pcyl_t.z;
                data[8*j+4] = color_f;
            }
            
            #pragma GCC ivdep
            for(int j = num_cols; j < num_pts; ++j) //Vectorization requires -fno-trapping-math
            {   
                cv::Point3f point(x[j],y[j],z[j]);
                cv::Point3f Pcyl_t = points.projectWorldToCan(point); 

                
                cv::Point3f normal(norm_x[j], norm_y[j], norm_z[j]);

                a = (std::sqrt( normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) > 0 ? 255 : 0);

                r = uint8_t(std::abs(normal.x) * 255);
                g = uint8_t(std::abs(normal.y) * 255);
                b = uint8_t(std::abs(normal.z) * 255);

                std::uint32_t rgba = ((std::uint32_t)a << 24 | (std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
                float color_f = *reinterpret_cast<float*>(&rgba);

                data[8*j] =   Pcyl_t.x;
                data[8*j+1] = Pcyl_t.y;
                data[8*j+2] = Pcyl_t.z;
                data[8*j+4] = color_f; 
            }
            
            

            pcloud_msg->header = points.getHeader();
            
            return pcloud_msg;
        
        }
        
    }   
}

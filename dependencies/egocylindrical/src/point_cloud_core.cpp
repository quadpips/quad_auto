#include <egocylindrical/point_cloud_core.h>

#include <egocylindrical/ecwrapper.h>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>

#include <omp.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>


namespace egocylindrical
{
    
    namespace utils
    {
        /* TODO: ensure that the right size 'ints' are used everywhere. On my current system, the size of int = minimum size of long int, so 32 bit system might fail */
        sensor_msgs::msg::PointCloud2::SharedPtr generate_point_cloud(const utils::ECWrapper& points)
        {
            const int num_cols = points.getNumPts();
            
            pcl::PointCloud<pcl::PointXYZ> pcloud;
            
            
            sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            pcl::toROSMsg(pcloud, *pcloud_msg);
            
            pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_cols);
            
            pcloud_msg->width = num_cols; //points.getWidth();
            pcloud_msg->height = 1;//points.getHeight();
            pcloud_msg->row_step = static_cast<uint32_t> (sizeof (pcl::PointXYZ) * pcloud_msg->width);
            pcloud_msg->is_dense = false;  //should be false, but seems to work with true
            
            const float* x = points.getX();
            const float* y = points.getY();
            const float* z = points.getZ();
            
            float* data = (float*) pcloud_msg->data.data();
        
            #pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
            //#pragma omp simd // schedule(static) num_threads(2)
            for(int j = 0; j < num_cols; ++j)
            {   
                data[4*j] = x[j];
                data[4*j+1] = y[j];
                data[4*j+2] = z[j];
                data[4*j+3] = 1;
            }

            pcloud_msg->header = points.getHeader();
            
            return pcloud_msg;
        
        }
        
    }   
}

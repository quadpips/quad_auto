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
        sensor_msgs::msg::PointCloud2::ConstSharedPtr generate_projected_point_cloud(const utils::ECWrapper& points)
        {
            const int num_pts = points.getNumPts();
          
            const int num_cols = points.getCols();
            
            pcl::PointCloud<pcl::PointXYZI> pcloud;
            
            sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            pcl::toROSMsg(pcloud, *pcloud_msg);
            
            pcloud_msg->data.resize(sizeof(pcl::PointXYZI) * num_pts);
            
            pcloud_msg->width = num_pts;
            pcloud_msg->height = 1;
            
            const float* __restrict__ x = points.getX();
            const float* __restrict__ y = points.getY();
            const float* __restrict__ z = points.getZ();
            
            float* data = (float*) pcloud_msg->data.data();

            //#pragma omp parallel for num_threads(4)
            #pragma GCC ivdep
            for(int j = 0; j < num_cols; ++j) //Vectorization requires -fno-trapping-math -fno-math-errno
            {   
                cv::Point3f point(x[j],y[j],z[j]);
                cv::Point3f Pcyl_t = points.projectWorldToCylinder(point);
                float range = egocylindrical::utils::worldToRange(point);

                data[8*j] =   Pcyl_t.x;
                data[8*j+1] = Pcyl_t.y;
                data[8*j+2] = Pcyl_t.z;
                data[8*j+4] = range;
            }
            
            #pragma GCC ivdep
            for(int j = num_cols; j < num_pts; ++j) //Vectorization requires -fno-trapping-math
            {   
                cv::Point3f point(x[j],y[j],z[j]);
                cv::Point3f Pcyl_t = points.projectWorldToCan(point); 
                float range = egocylindrical::utils::worldToCanDepth(point);

                data[8*j] =   Pcyl_t.x;
                data[8*j+1] = Pcyl_t.y;
                data[8*j+2] = Pcyl_t.z;
                data[8*j+4] = range;
            }
            
            

            pcloud_msg->header = points.getHeader();
            
            return pcloud_msg;
        
        }
        
    }   
}

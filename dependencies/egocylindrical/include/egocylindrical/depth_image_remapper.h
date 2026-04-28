#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_REMAPPER_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_REMAPPER_H

#include <egocylindrical/depth_image_common.h>
#include <egocylindrical/ecwrapper.h>

#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace egocylindrical
{
    namespace utils
    {
        
        class DepthImageRemapper
        {
        private:
            AlignedVector<long int> inds_;
            AlignedVector<float> x_;
            AlignedVector<float> y_;
            AlignedVector<float> z_;  //TODO: Remove all references to z_

            int num_pixels_;
            
            ECParams ec_params_;
          
            CleanCameraModel cam_model_;
            
        public:
            
            //inline
            void update( ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);
            
            //inline
            void update( ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, float thresh_min, float thresh_max);
            
        private:
          
            //inline
            void updateMapping(const ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);
            
//             template <bool fill_cloud>
//             inline
//             void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, float thresh_min, float thresh_max);
            
            //template <bool fill_cloud>
            //inline
            void update( ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, float thresh_min, float thresh_max, bool fill_cloud);
            
        };
      
    } //end namespace utils

} //end namespace egocylindrical


#endif //EGOCYLINDRICAL_DEPTH_IMAGE_REMAPPER_H

#include <egocylindrical/depth_image_remapper.h>

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

      
        // TODO: use coordinate converter instead, since we don't need the actual data here
        template <typename T, typename U, typename S>
        inline
        void initializeDepthMapping(const utils::ECWrapper& cylindrical_history, const image_geometry::PinholeCameraModel& cam_model, U* inds, S* x, S* y, S* z)
        {
            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            uint scale = DepthScale<T>::scale();
                                    
            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {   
                    int image_idx = i*image_width + j;
                    
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    cv::Point3f world_pnt = cam_model.projectPixelTo3dRay(pt);
                    
                    int cyl_idx = cylindrical_history.worldToIdx(world_pnt);  //Control flow, can't vectorize
                    
                    inds[image_idx] = cyl_idx;
                    
                    x[image_idx] = world_pnt.x/scale;
                    y[image_idx] = world_pnt.y/scale;
                }
            }
            
        }

        
        template <typename U, typename S>
        inline
        void initializeDepthMapping(const utils::ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const image_geometry::PinholeCameraModel& cam_model, U* inds, S* x, S* y, S* z)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
            
            if(image.depth() == CV_32FC1)
            {
                initializeDepthMapping<float>(cylindrical_points, cam_model, inds, x, y, z);
            }
            else if (image.depth() == CV_16UC1)
            {
                initializeDepthMapping<uint16_t>(cylindrical_points, cam_model, inds, x, y, z);
            }
        }

        
        template <typename T, bool fill_cloud, typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const T* depths, const U* inds, const S* n_x, const S* n_y, const S* n_z, int num_pixels, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, S thresh_min, S thresh_max)
        {

            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            
            float* data;
            if(fill_cloud)
            {
                pcl::PointCloud<pcl::PointXYZ> pcloud;
                pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(pcloud, *pcloud_msg);
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_pixels);
                pcloud_msg->is_dense = true;  //should be false, but seems to work with true
                data = (float*) pcloud_msg->data.data();
            }
            
            int j = 0;
            
            //Contents of 'inds' are not sequential, can't vectorize
            for(int i = 0; i < num_pixels; ++i)
            {
                
                T depth = depths[i];
                
                if(depth>0)
                {
                    // NOTE: Currently, no check that index is in bounds. As long as the camera's fov fits inside the egocylindrical fov, this is safe
                    U idx = inds[i];
                    
                    S x_val = n_x[i]*depth;
                    S y_val = n_y[i]*depth;
                    S z_val =  ((S) depth)/DepthScale<T>::scale();  //NOTE: This should result in simply z_val=depth (for float) and z_val=depth/1000 (for uint16)
                    
                    if(fill_cloud && (y_val > thresh_min) && (y_val < thresh_max))
                    {
                        data[4*j] = x_val;;
                        data[4*j+1] = y_val;
                        data[4*j+2] = z_val;
                        data[4*j+3] = 1;  //Not necessary, only include if improves performance
                        
                        ++j;
                    }
                    
                    if(idx >=0)
                    {
                    
                        if (!(z[idx] <= z_val))
                        {
                            x[idx] = x_val;
                            y[idx] = y_val;
                            z[idx] = z_val;
                        }
                    }
                }
            }
            
            if(fill_cloud)
            {
                pcloud_msg->width = j;
                pcloud_msg->height = 1;
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ)*j);
                pcloud_msg->row_step = static_cast<uint32_t> (sizeof (pcl::PointXYZ) * pcloud_msg->width);
            }
            
        }
        
        template <bool fill_cloud, typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const cv::Mat& image, const U* inds, const S* n_x, const S* n_y, const S* n_z, int num_pixels, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, S thresh_min, S thresh_max)
        {
            if(image.depth() == CV_32FC1)
            {
                remapDepthImage<float,fill_cloud>(cylindrical_points, (const float*)image.data, inds, n_x, n_y, n_z, num_pixels, pcloud_msg, thresh_min, thresh_max);
            }
            else if (image.depth() == CV_16UC1)
            {
                remapDepthImage<uint16_t,fill_cloud>(cylindrical_points, (const uint16_t*)image.data, inds, n_x, n_y, n_z, num_pixels, pcloud_msg, thresh_min, thresh_max);
            }
        }
        
        template <bool fill_cloud, typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const U* inds, const S* n_x, const S* n_y, const S* n_z, int num_pixels, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, S thresh_min, S thresh_max)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
            remapDepthImage<fill_cloud>(cylindrical_points, image, inds, n_x, n_y, n_z, num_pixels, pcloud_msg, thresh_min, thresh_max);
        }
        

        
        //DepthImageRemapper definitions
        
        inline
        void DepthImageRemapper::updateMapping(const ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
        {
            bool reinit = false;
            
            if( cam_model_.fromCameraInfo(cam_info) )
            {
                cam_model_.init();
                
                cv::Size image_size = cam_model_.reducedResolution();
                int image_width = image_size.width;
                int image_height = image_size.height;
                
                num_pixels_ = image_width * image_height;
                inds_.resize(num_pixels_);
                x_.resize(num_pixels_);
                y_.resize(num_pixels_);
                //z_.resize(num_pixels_);
                
                // ROS_INFO("Camera parameters changed");
                
                reinit=true;
            }
            
            ECParams new_params = cylindrical_points.getParams();
            if(!(ec_params_ == new_params))
            {
                // ROS_INFO("Egocylindrical parameters changed");
                
                ec_params_ = new_params;
                
                reinit = true;
            }
            
            if(reinit)
            {
                
                // ROS_INFO("Generating depth to cylindrical image mapping");
                initializeDepthMapping(cylindrical_points, image_msg, cam_model_, inds_.data(), x_.data(), y_.data(), z_.data());
            }
        }
        
//         template <bool fill_cloud>
//         inline
//         void DepthImageRemapper::remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, float thresh_min, float thresh_max)
//         {
//             utils::remapDepthImage<fill_cloud>(cylindrical_points, image_msg, inds_.data(), x_.data(), y_.data(), z_.data(), num_pixels_, pcloud_msg, thresh_min, thresh_max);
//         }
//         
        //template <bool fill_cloud>
        inline
        void DepthImageRemapper::update( ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, float thresh_min, float thresh_max, bool fill_cloud)
        {
            // ROS_DEBUG("Updating cylindrical points with depth image");
            
            // ros::WallTime// start = ros::WallTime::now();
            updateMapping( cylindrical_points, image_msg, cam_info);
            // ros::WallTimemid = ros::WallTime::now();
            
            if(fill_cloud)
            {
                utils::remapDepthImage<true>(cylindrical_points, image_msg, inds_.data(), x_.data(), y_.data(), z_.data(), num_pixels_, pcloud_msg, thresh_min, thresh_max);            }
            else
            {
                utils::remapDepthImage<false>(cylindrical_points, image_msg, inds_.data(), x_.data(), y_.data(), z_.data(), num_pixels_, pcloud_msg, thresh_min, thresh_max);            }
            // ros::WallTimeend = ros::WallTime::now();
            
            // ROS_DEBUG_STREAM_NAMED("timing", "Updating camera model took " <<  (mid - start).toSec() * 1e3 << "ms");
            // ROS_DEBUG_STREAM_NAMED("timing", "Remapping depth image took " <<  (end - mid).toSec() * 1e3 << "ms");
            
        }
        
        //inline
        void DepthImageRemapper::update( ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
        {
            float thresh_min = 0;
            float thresh_max = 0;
            sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg;
            update(cylindrical_points, image_msg, cam_info, pcloud_msg, thresh_min, thresh_max, false);
        }
        
        //inline
        void DepthImageRemapper::update( ECWrapper& cylindrical_points, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info, sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg, float thresh_min, float thresh_max)
        {
            update(cylindrical_points, image_msg, cam_info, pcloud_msg, thresh_min, thresh_max, true);
            pcloud_msg->header = image_msg->header;
        }

    } //end namespace utils
    
} //end namespace egocylindrical

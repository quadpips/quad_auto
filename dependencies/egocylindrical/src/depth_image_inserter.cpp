#include <egocylindrical/depth_image_inserter.h>
// #include "depth_image_difference.cpp"

#include <egocylindrical/point_transformer_object.h>
#include <egocylindrical/depth_image_common.h>
#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/depth_image_difference.h>

// #include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cstdlib>

namespace egocylindrical
{
    namespace utils
    {
        //whole image vectorization w/ inds
        template <typename T>
        void insertPoints4(utils::ECWrapper& cylindrical_points, 
                            const cv::Mat & image, 
                            const CleanCameraModel& cam_model, 
                            const geometry_msgs::msg::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const bool use_egocan = cylindrical_points.getParams().can_width>0;
            
            const int max_ind = cylindrical_points.getCols();
            float* __restrict__ x = cylindrical_points.getX(); // size: getNumPt()
            float* __restrict__ y = cylindrical_points.getY(); // size: getNumPt()
            float* __restrict__ z = cylindrical_points.getZ(); // size: getNumPt()

            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), // size: num_pixels
                                    nx(num_pixels, dNaN), // size: num_pixels
                                    ny(num_pixels, dNaN), // size: num_pixels
                                    nz(num_pixels, dNaN); // size: num_pixels

            AlignedVector<int32_t> inds(num_pixels);
            
            // ROS_INFO_STREAM_NAMED("timing", "getNumPts:" << cylindrical_points.getNumPts());
            // ROS_INFO_STREAM_NAMED("timing", "num_pixels:" << num_pixels);

            const T* const __restrict__ imgptr = (T* const) image.data;

            const float row_factor = ((float) 1)/image_width;
            
            #pragma GCC ivdep
            for(int i = 0; i < num_pixels; ++i)
            {
                float raw_row = i*row_factor;
                float row = std::floor(raw_row);

                float decimal = raw_row - row;
                float col = decimal * image_width;
                //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                //int row = i / image_width;
                //int col = i % image_width;

                
                T depth = imgptr[i];
                
                cv::Point2d pt(col, row);

                if (depth == 0)
                {
                    depth = 5.0 * scale; // treat 0 depth as max range
                }

                //if(depth>0)  //Only insert actual points (works for both float and uint16)
                {                        
                    cv::Point3f ray = cam_model.projectPixelTo3dRay(pt); // compute image ray
                    cv::Point3f world_pnt = ray * (((float) depth)/scale); // scale image ray by depth to get world point
                    cv::Point3f transformed_pnt = point_transformer.transform(world_pnt); // transform between prior timestep and current timestep (or something, not so sure)

                    nx[i] = transformed_pnt.x;
                    ny[i] = transformed_pnt.y;
                    nz[i] = transformed_pnt.z;

                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                    
                    float range_sq = worldToRangeSquared(transformed_pnt);
                    
                    //Only insert actual points (works for both float and uint16)
                    ranges[i] = (depth>0) ? range_sq : dNaN;    
                    inds[i] = cyl_idx;
                }                
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    
                    int cyl_idx = inds[i];
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind) // mapping onto columns of egocylinder
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);

                        if(!(prev_range_sq <= range_sq)) //overwrite || 
                        {   
                            x[cyl_idx] = transformed_pnt.x;
                            y[cyl_idx] = transformed_pnt.y;
                            z[cyl_idx] = transformed_pnt.z;
                        }
                    }
                    else if (use_egocan)
                    {
                        cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                    
                        if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts()) // mapping onto top/bottom of egocan
                        {
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
                            {                               
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;
                            }
                        }
                    }
                }
            }
            
        }

        // whole image vectorization w/ inds
        template <typename T>
        void insertPoints4(utils::ECWrapper& cylindrical_points, 
                            const cv::Mat & image, 
                            const cv::Mat & normals,
                            const CleanCameraModel& cam_model, 
                            const geometry_msgs::msg::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const bool use_egocan = cylindrical_points.getParams().can_width>0;
            
            const int max_ind = cylindrical_points.getCols();
            float* __restrict__ x = cylindrical_points.getX(); // size: getNumPt()
            float* __restrict__ y = cylindrical_points.getY(); // size: getNumPt()
            float* __restrict__ z = cylindrical_points.getZ(); // size: getNumPt()

            float* __restrict__ norm_x = cylindrical_points.getNormalX(); // size: getNumPt()
            float* __restrict__ norm_y = cylindrical_points.getNormalY(); // size: getNumPt()
            float* __restrict__ norm_z = cylindrical_points.getNormalZ(); // size: getNumPt()
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), // size: num_pixels
                                    nx(num_pixels, dNaN), // size: num_pixels
                                    ny(num_pixels, dNaN), // size: num_pixels
                                    nz(num_pixels, dNaN), // size: num_pixels
                                    n_norm_x(num_pixels, dNaN), // size: num_pixels
                                    n_norm_y(num_pixels, dNaN), // size: num_pixels
                                    n_norm_z(num_pixels, dNaN); // size: num_pixels

            AlignedVector<int32_t> inds(num_pixels);
            
            // ROS_INFO_STREAM_NAMED("timing", "getNumPts:" << cylindrical_points.getNumPts());
            // ROS_INFO_STREAM_NAMED("timing", "num_pixels:" << num_pixels);

            const T* const __restrict__ imgptr = (T* const) image.data;

            const float row_factor = ((float) 1)/image_width;
            
            #pragma GCC ivdep
            for(int i = 0; i < num_pixels; ++i)
            {
                float raw_row = i*row_factor;
                float row = std::floor(raw_row);

                float decimal = raw_row - row;
                float col = decimal * image_width;
                //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                //int row = i / image_width;
                //int col = i % image_width;

                
                T depth = imgptr[i];
                
                cv::Point2d pt(col, row);

                //if(depth>0)  //Only insert actual points (works for both float and uint16)
                {                        
                    cv::Point3f ray = cam_model.projectPixelTo3dRay(pt); // compute image ray
                    cv::Point3f world_pnt = ray * (((float) depth)/scale); // scale image ray by depth to get world point
                    cv::Point3f transformed_pnt = point_transformer.transform(world_pnt); // transform between prior timestep and current timestep (or something, not so sure)
                    
                    cv::Point3f world_norm = cv::Point3f(normals.at<cv::Vec3f>(row, col)[0], 
                                                            normals.at<cv::Vec3f>(row, col)[1], 
                                                            normals.at<cv::Vec3f>(row, col)[2]);
                    cv::Point3f transformed_norm = point_transformer.rotate(world_norm);

                    // cv::Point3f world_pnt_plus_norm = world_pnt + world_norm;
                    // cv::Point3f transformed_pnt_plus_norm = point_transformer.transform(world_pnt_plus_norm); // transform between prior timestep and current timestep (or something, not so sure)

                    // cv::Point3f transformed_norm = transformed_pnt_plus_norm - transformed_pnt;
                    // cv::Point3f re_normalized_norm = transformed_norm / cv::norm(transformed_norm);


                    nx[i] = transformed_pnt.x;
                    ny[i] = transformed_pnt.y;
                    nz[i] = transformed_pnt.z;

                    n_norm_x[i] = transformed_norm.x;
                    n_norm_y[i] = transformed_norm.y;
                    n_norm_z[i] = transformed_norm.z;

                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                    
                    float range_sq = worldToRangeSquared(transformed_pnt);
                    
                    //Only insert actual points (works for both float and uint16)
                    ranges[i] = (depth>0) ? range_sq : dNaN;    
                    inds[i] = cyl_idx;
                }                
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    cv::Point3f transformed_norm(n_norm_x[i], n_norm_y[i], n_norm_z[i]);
                    
                    int cyl_idx = inds[i];
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind) // mapping onto columns of egocylinder
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);

                        if(!(prev_range_sq <= range_sq)) //overwrite || 
                        {   
                            x[cyl_idx] = transformed_pnt.x;
                            y[cyl_idx] = transformed_pnt.y;
                            z[cyl_idx] = transformed_pnt.z;

                            norm_x[cyl_idx] = transformed_norm.x;
                            norm_y[cyl_idx] = transformed_norm.y;
                            norm_z[cyl_idx] = transformed_norm.z;
                        }
                    }
                    else if (use_egocan)
                    {
                        cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                    
                        if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts()) // mapping onto top/bottom of egocan
                        {
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
                            {                               
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;

                                norm_x[cyl_idx] = transformed_norm.x;
                                norm_y[cyl_idx] = transformed_norm.y;
                                norm_z[cyl_idx] = transformed_norm.z;
                            }
                        }
                    }
                }
            }
        }
      
        // whole image vectorization w/ inds
        template <typename T>
        void insertLabelsAndPoints4(utils::ECWrapper& cylindrical_points, 
                                    const cv::Mat & image, 
                                    const cv::Mat & normals,
                                    const cv::Mat & step_labels, 
                                    const CleanCameraModel& cam_model, 
                                    const geometry_msgs::msg::TransformStamped & transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const bool use_egocan = cylindrical_points.getParams().can_width>0;
            
            const int max_ind = cylindrical_points.getCols();
            float* __restrict__ x = cylindrical_points.getX(); // size: getNumPt()
            float* __restrict__ y = cylindrical_points.getY(); // size: getNumPt()
            float* __restrict__ z = cylindrical_points.getZ(); // size: getNumPt()
            float* __restrict__ norm_x = cylindrical_points.getNormalX(); // size: getNumPt()
            float* __restrict__ norm_y = cylindrical_points.getNormalY(); // size: getNumPt()
            float* __restrict__ norm_z = cylindrical_points.getNormalZ(); // size: getNumPt()
            uint8_t* __restrict__ labels = cylindrical_points.getLabels(); // size: getNumPt()
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), // size: num_pixels
                                    nx(num_pixels, dNaN), // size: num_pixels
                                    ny(num_pixels, dNaN), // size: num_pixels
                                    nz(num_pixels, dNaN), // size: num_pixels
                                    n_norm_x(num_pixels, dNaN), // size: num_pixels
                                    n_norm_y(num_pixels, dNaN), // size: num_pixels
                                    n_norm_z(num_pixels, dNaN); // size: num_pixels

            AlignedVector<uint8_t> nlabels(num_pixels); // size: num_pixels
            AlignedVector<int32_t> inds(num_pixels);
            
            // ROS_INFO_STREAM_NAMED("timing", "getNumPts:" << cylindrical_points.getNumPts());
            // ROS_INFO_STREAM_NAMED("timing", "num_pixels:" << num_pixels);

            const T* const __restrict__ imgptr = (T* const) image.data;
            const uint8_t* const __restrict__ labelsptr = (uint8_t* const) step_labels.data; // do not do!!
            
            // // ROS_INFO_STREAM_NAMED("timing", "step_labels cols: " << step_labels.cols);
            // // ROS_INFO_STREAM_NAMED("timing", "step_labels rows: " << step_labels.rows);
            // // ROS_INFO_STREAM_NAMED("timing", "step_labels size: " << step_labels.size);
            // // ROS_INFO_STREAM_NAMED("timing", "nlabels[200000]: " << nlabels[200000]);

            // ROS_INFO_STREAM_NAMED("labels", "step_labels.data[100] " << std::hex << (uint16_t) step_labels.data[100]);

            // // ROS_INFO_STREAM_NAMED("timing", "imgptr[2]: " << imgptr[2]);

            // nlabels[200000] = labelsptr[200000];

            const float row_factor = ((float) 1)/image_width;
            
            #pragma GCC ivdep
            for(int i = 0; i < num_pixels; ++i)
            {
                float raw_row = i*row_factor;
                float row = std::floor(raw_row);

                float decimal = raw_row - row;
                float col = decimal * image_width;
                //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                //int row = i / image_width;
                //int col = i % image_width;

                
                T depth = imgptr[i];
                uint8_t label = labelsptr[i]; // step_labels.at<uint8_t>(row_int, col_int); // step_labels.data[100000]; //                 

                cv::Point2d pt(col, row);

                //if(depth>0)  //Only insert actual points (works for both float and uint16)
                {                        
                    cv::Point3f ray = cam_model.projectPixelTo3dRay(pt); // compute image ray
                    cv::Point3f world_pnt = ray * (((float) depth)/scale); // scale image ray by depth to get world point
                    cv::Point3f transformed_pnt = point_transformer.transform(world_pnt); // transform between prior timestep and current timestep (or something, not so sure)
                    
                    cv::Point3f world_norm = cv::Point3f(normals.at<cv::Vec3f>(row, col)[0], 
                                                            normals.at<cv::Vec3f>(row, col)[1], 
                                                            normals.at<cv::Vec3f>(row, col)[2]);
                    cv::Point3f transformed_norm = point_transformer.rotate(world_norm);

                    // cv::Point3f world_pnt_plus_norm = world_pnt + world_norm;
                    // cv::Point3f transformed_pnt_plus_norm = point_transformer.transform(world_pnt_plus_norm); // transform between prior timestep and current timestep (or something, not so sure)

                    // cv::Point3f transformed_norm = transformed_pnt_plus_norm - transformed_pnt;
                    // cv::Point3f re_normalized_norm = transformed_norm / cv::norm(transformed_norm);

                    nx[i] = transformed_pnt.x;
                    ny[i] = transformed_pnt.y;
                    nz[i] = transformed_pnt.z;

                    n_norm_x[i] = transformed_norm.x;
                    n_norm_y[i] = transformed_norm.y;
                    n_norm_z[i] = transformed_norm.z;
                    // // ROS_INFO_STREAM_NAMED("timing", "nlabels size: " << nlabels.size());
                    // // ROS_INFO_STREAM_NAMED("timing", "i: " << i);

                    // nlabels[i] = 0; // Fine.

                    nlabels[i] = label;  // step_labels.data[i]; // Fine now.                
                       
                    // if (i == 100)
                    // {
                    //     // // ROS_DEBUG_STREAM_NAMED("labels", "label:" << label);
                    //     // ROS_DEBUG_STREAM_NAMED("labels", "nlabels[" << i << "]:" << std::hex << (uint16_t) nlabels[i]);                     
                    //     // // ROS_DEBUG_STREAM_NAMED("labels", "nlabels[" << i << "]:" << nlabels[i]);
                    // }

                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                    
                    float range_sq = worldToRangeSquared(transformed_pnt);
                    
                    //Only insert actual points (works for both float and uint16)
                    ranges[i] = ( depth > 0) ? range_sq : dNaN;    
                    inds[i] = cyl_idx;
                }                
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if (ranges[i] > 0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    cv::Point3f transformed_norm(n_norm_x[i], n_norm_y[i], n_norm_z[i]);
                    
                    int cyl_idx = inds[i];

                    if (cyl_idx >= 0  && cyl_idx < max_ind) // mapping onto columns of egocylinder
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);

                        if( !(prev_range_sq <= range_sq)) //overwrite || 
                        {   
                            x[cyl_idx] = transformed_pnt.x;
                            y[cyl_idx] = transformed_pnt.y;
                            z[cyl_idx] = transformed_pnt.z;

                            norm_x[cyl_idx] = transformed_norm.x;
                            norm_y[cyl_idx] = transformed_norm.y;
                            norm_z[cyl_idx] = transformed_norm.z;
                            
                            labels[cyl_idx] = nlabels[i]; // setting i for i does not feel right to me.
                        }
                    }
                    else if (use_egocan)
                    {
                        cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                    
                        if (cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts()) // mapping onto top/bottom of egocan
                        {
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if (!(prev_can_depth <= can_depth)) //overwrite || 
                            {                               
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;

                                norm_x[cyl_idx] = transformed_norm.x;
                                norm_y[cyl_idx] = transformed_norm.y;
                                norm_z[cyl_idx] = transformed_norm.z;

                                labels[cyl_idx] = nlabels[i]; // setting i for i does not feel right to me.
                                if (i == 100)
                                {
                                    // // ROS_DEBUG_STREAM_NAMED("labels", "nlabels[" << i << "]:" << nlabels[i]);
                                    // ROS_DEBUG_STREAM_NAMED("labels", "labels[" << cyl_idx << "]:" << std::hex << (uint16_t) labels[cyl_idx]);                     
                                    // // ROS_DEBUG_STREAM_NAMED("labels", "labels[" << cyl_idx << "]:" << labels[cyl_idx]);
                                }                                
                                // // ROS_DEBUG_STREAM_NAMED("labels", "labels[cyl_idx]: " << labels[cyl_idx]);

                            }
                        }
                    }
                }
            }
        }

        //whole image vectorization
        template <typename T>
        void insertPoints3(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::msg::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), nx(num_pixels, dNaN), ny(num_pixels, dNaN), nz(num_pixels, dNaN);
            AlignedVector<uint32_t> inds(num_pixels);
            
            const T* const imgptr = (T* const) image.data;
            
            const float row_factor = ((float) 1)/image_width;
            
            for(int i = 0; i < num_pixels; ++i)
            {
                float raw_row = i*row_factor;
                float row = std::floor(raw_row);
                float decimal = raw_row - row;
                float col = decimal * image_width;
                //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                //int row = i / image_width;
                //int col = i % image_width;

                
                T depth = imgptr[i];
                
                cv::Point2d pt(col, row);

                //if(depth>0)  //Only insert actual points (works for both float and uint16)
                {                        
                    cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                    cv::Point3f world_pnt = ray * (((float) depth)/scale);
                    cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                    
                    nx[i] = transformed_pnt.x;
                    ny[i] = transformed_pnt.y;
                    nz[i] = transformed_pnt.z;
                    
                    //int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                    
                    float range_sq = worldToRangeSquared(transformed_pnt);
                    
                    //Only insert actual points (works for both float and uint16)
                    ranges[i] = (depth>0) ? range_sq : dNaN;
                }                
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    
                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind)
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);
                        
                        if(!(prev_range_sq <= range_sq)) //overwrite || 
                        {   
                            x[cyl_idx] = transformed_pnt.x;
                            y[cyl_idx] = transformed_pnt.y;
                            z[cyl_idx] = transformed_pnt.z;
                        }
                    }
                    else
                    {
                        cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                    
                        if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts())
                        {
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
                            {   
                                
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;
                            }
                        }
                    }
                }
            }
            
        }
        
        //row-wise vectorization
        template <typename T>
        void insertPoints2(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::msg::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), nx(num_pixels, dNaN), ny(num_pixels, dNaN), nz(num_pixels, dNaN);
            
            const T* const imgptr = (T* const) image.data;
            
            //for(int i = 0; i < num_pixels; ++i)
            //{
            int i = 0;
            for(int row = 0; row < image_height; ++row)
            {
                for(int col = 0; col < image_width; ++col)
                {                       
                    //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                    //int row = i / image_width;
                    //int col = i % image_width;
                    
                    //int row = i+10;
                    //int col = i + 30;
                    
                    T depth = imgptr[i];
                    
                    cv::Point2d pt(col, row);

                    {                        
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (((float) depth)/scale);
                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        nx[i] = transformed_pnt.x;
                        ny[i] = transformed_pnt.y;
                        nz[i] = transformed_pnt.z;
                        
                        //int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        
                        float range_sq = worldToRangeSquared(transformed_pnt);
                        
                        //Only insert actual points (works for both float and uint16)
                        ranges[i] = (depth>0) ? range_sq : dNaN;
                    }
                    i++;
                }
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    
                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind)
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);
                        
                        if(!(prev_range_sq <= range_sq)) //overwrite || 
                        {   
                            x[cyl_idx] = transformed_pnt.x;
                            y[cyl_idx] = transformed_pnt.y;
                            z[cyl_idx] = transformed_pnt.z;
                        }
                    }
                    else
                    {
                        cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                    
                        if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts())
                        {
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
                            {   
                                
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;
                            }
                        }
                    }
                }
            }
            
        }
      
        //immediate insert
        template <typename T>
        void insertPoints(utils::ECWrapper& cylindrical_points, 
                            const cv::Mat image, 
                            const CleanCameraModel& cam_model, 
                            const geometry_msgs::msg::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
                                    
            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {                       
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    T depth = image.at<T>(i,j);
                    
                    
                    if(depth>0)  //Only insert actual points (works for both float and uint16)
                    {                        
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (((float) depth)/scale);
                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        
                        if(cyl_idx >= 0  && cyl_idx < max_ind)
                        {
                            float range_sq = worldToRangeSquared(transformed_pnt);
                    
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_range_sq = worldToRangeSquared(prev_point);
                            
                            if(!(prev_range_sq <= range_sq)) //overwrite || 
                            {   
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;
                            }
                        }
                        else
                        {
                            cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                        
                            if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts())
                            {
                                float can_depth = worldToCanDepth(transformed_pnt);
                                
                                cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                                
                                float prev_can_depth = worldToCanDepth(prev_point);
                                
                                if(!(prev_can_depth <= can_depth)) //overwrite || 
                                {   
                                    x[cyl_idx] = transformed_pnt.x;
                                    y[cyl_idx] = transformed_pnt.y;
                                    z[cyl_idx] = transformed_pnt.z;
                                }
                            }
                        }
                    }
                    
                }
            }
        }

        inline
        void insertPoints(utils::ECWrapper& cylindrical_points, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                            const CleanCameraModel& cam_model, 
                            const geometry_msgs::msg::TransformStamped transform, 
                            DIDiffRequest& request)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;

            // insertPoints6(cylindrical_points, image, image_msg, cam_model, transform, request);

            if(image.depth() == CV_32FC1)
            {
                insertPoints4<float>(cylindrical_points, image, cam_model, transform); // labels, 
            }
            else if (image.depth() == CV_16UC1)
            {
                insertPoints4<uint16_t>(cylindrical_points, image, cam_model, transform); // labels, 
            }
        }

        inline
        void insertPoints(utils::ECWrapper& cylindrical_points, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& normals_msg,
                            const CleanCameraModel& cam_model, 
                            const geometry_msgs::msg::TransformStamped transform, 
                            DIDiffRequest& request)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
            cv_bridge::CvImageConstPtr normalsPtr = cv_bridge::toCvCopy(normals_msg, "32FC3");
            const cv::Mat normals = normalsPtr->image;

            // // ROS_INFO_STREAM_NAMED("timing", "in DepthImageInserter::insertPoints, normals.at(240, 360): " << 
            //                                                                     normals.at<cv::Vec3f>(240, 360)[0] << ", " <<
            //                                                                     normals.at<cv::Vec3f>(240, 360)[1] << ", " <<
            //                                                                     normals.at<cv::Vec3f>(240, 360)[2]);

            if (image.depth() == CV_32FC1)
            {
                insertPoints4<float>(cylindrical_points, image, normals, cam_model, transform); //   
            }
            else if (image.depth() == CV_16UC1)
            {
                insertPoints4<uint16_t>(cylindrical_points, image, normals, cam_model, transform); //   
            }
        }

        inline
        void insertPoints(utils::ECWrapper& cylindrical_points, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                            const sensor_msgs::msg::Image::ConstSharedPtr& normals_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr& labels_msg, 
                            const CleanCameraModel& cam_model, 
                            const geometry_msgs::msg::TransformStamped transform, 
                            DIDiffRequest& request)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;

            // const cv::Mat labels = cv_bridge::toCvCopy(labels_msg, "8UC1")->image; // do not do!
            cv_bridge::CvImageConstPtr cvImagePtr = cv_bridge::toCvCopy(labels_msg, "8UC1"); // , "8UC1"
            const cv::Mat labels = cvImagePtr->image;

            cv_bridge::CvImageConstPtr normalsPtr = cv_bridge::toCvCopy(normals_msg, "32FC3");
            const cv::Mat normals = normalsPtr->image;

            // // ROS_INFO_STREAM_NAMED("labels", "labels: " << labels.data);

            // uint8_t test_label_uint8_t = labels.at<uint8_t>(0, 100); // DOES NOT WORK 
            // int test_label_int = labels.at<uint8_t>(0, 100); // WORKS
            // int8_t test_label_int8_t = labels.at<int8_t>(0, 100); // DOES NOT WORK 
            // int test_label_int2 = labels.at<int8_t>(0, 100); // WORKS

            // uint16_t test_label_uint16_t = labels.at<uint16_t>(0, 100); // WORKS, returns 771
            // uint8_t test_16_to_8_convert = static_cast<uint8_t>((test_label_uint16_t & 0xFF00) >> 8);

            // int test_label_data_int = labels.data[100]; // WORKS
            // int8_t test_label_data_int8_t = labels.data[100]; // DOES NOT WORK

            // uint8_t * test_convert_uint8 = (uint8_t*) &test_label_data_int; // DOES NOT WORK
            // int8_t * test_convert_int8 = (int8_t*) &test_label_data_int; // DOES NOT WORK

            // short test_convert_short = labels.at<uint8_t>(0, 100); // WORKS
            // float test_convert_float = labels.at<uint8_t>(0, 100); // WORKS

            // uint8_t test_label_uint8_t_int = labels.at<int>(0, 100); // DOES NOT WORK 

            // if (test_label_uint8_t == 3)
            //     // ROS_INFO_STREAM_NAMED("labels", "COMPARING IS FINE!!");    

            // // ROS_INFO_STREAM_NAMED("labels", "test_label_uint8_t: " << std::hex << (uint16_t) test_label_uint8_t);
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_int: " << test_label_int);
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_int8_t: " << test_label_int8_t);
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_int2: " << test_label_int2);
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_data: " << test_label_data_int);
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_data_int8_t: " << test_label_data_int8_t);
            // // ROS_INFO_STREAM_NAMED("labels", "test_convert_uint8: " << *test_convert_uint8);
            // // ROS_INFO_STREAM_NAMED("labels", "test_convert_int8: " << *test_convert_int8);
            // // ROS_INFO_STREAM_NAMED("labels", "test_convert_short: " << test_convert_short);
            // // ROS_INFO_STREAM_NAMED("labels", "test_convert_float: " << test_convert_float);
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<uint8_t>(0, 100): " << labels.at<uint8_t>(0, 100));
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<int8_t>(0, 100): " << labels.at<int8_t>(0, 100));
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<uint16_t>(0, 100): " << labels.at<uint16_t>(0, 100)); // huge number 
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<int16_t>(0, 100): " << labels.at<int16_t>(0, 100)); // huge number            
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<uint32_t>(0, 100): " << labels.at<uint32_t>(0, 100)); // huge number 
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<int32_t>(0, 100): " << labels.at<int32_t>(0, 100)); // huge number
            // // ROS_INFO_STREAM_NAMED("labels", "labels.at<int>(0, 100): " << labels.at<int>(0, 100));
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_uint8_t_int: " << test_label_uint8_t_int);
            // // ROS_INFO_STREAM_NAMED("labels", "test_label_uint16_t: " << test_label_uint16_t); 
            // // ROS_INFO_STREAM_NAMED("labels", "test_16_to_8_convert: " << test_16_to_8_convert); 

            // int label = labels.at<uint8_t>(3, 3);
            // // ROS_INFO_STREAM_NAMED("timing", "in DepthImageInserter::insertPoints, label: " << label);
            // // ROS_INFO_STREAM_NAMED("timing", "in DepthImageInserter::insertPoints, current use: " << labels.at<uint8_t>(3, 3));

            if(image.depth() == CV_32FC1)
            {
                insertLabelsAndPoints4<float>(cylindrical_points, image, normals, labels, cam_model, transform);  
            }
            else if (image.depth() == CV_16UC1)
            {
                insertLabelsAndPoints4<uint16_t>(cylindrical_points, image, normals, labels, cam_model, transform); 
            }
        }

        DepthImageInserter::DepthImageInserter(std::shared_ptr<tf2_ros::Buffer> buffer, rclcpp::Node::SharedPtr node):
            buffer_(buffer),
            node_(node)
            // pnh_(pnh)
        {}
        
        bool DepthImageInserter::init()
        {
            //TODO: use pips::param_utils
            std::string fixed_frame_id;
            // bool status = pnh_.getParam("fixed_frame_id", fixed_frame_id);
            fixed_frame_id_ = node_->get_parameter("fixed_frame_id", fixed_frame_id_);
            return init(fixed_frame_id_); // status && 
        }
        
        bool DepthImageInserter::init(std::string fixed_frame_id)
        {
            fixed_frame_id_ = fixed_frame_id;
            // pub_diff_pc_ = pnh_.advertise<sensor_msgs::msg::PointCloud2>("diff_points",2);
            // pub_diff_im_ = pnh_.advertise<sensor_msgs::msg::Image>("diff_im", 2);
            pub_diff_pc_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("diff_points", 2);
            pub_diff_im_ = node_->create_publisher<sensor_msgs::msg::Image>("diff_im", 2);

            debug_pub_.init(node_);
            return true;
        }

        bool DepthImageInserter::insert(ECWrapper& cylindrical_points, 
                                        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)                                        
        {
            const std_msgs::msg::Header& target_header = cylindrical_points.getHeader();
            const std_msgs::msg::Header& source_header = image_msg->header;

            if (target_header == source_header)
            {
                RCLCPP_DEBUG_STREAM(node_->get_logger(), "Target and source headers match, using remapping approach");
                // // ROS_DEBUG_STREAM_NAMED("labels", "we are remapping now!"); // not happening it appears
                depth_remapper_.update(cylindrical_points, image_msg, cam_info);
                return true;
            }
            
            if (cam_model_.fromCameraInfo(cam_info))
            {
                RCLCPP_DEBUG_STREAM(node_->get_logger(), "Camera info has changed!");
                //If camera info changed, update any precomputed values
                //cam_model_.init();
            }
            else
            {
                // ROS_DEBUG("Camera info has not changed");
            }
            
            //Get transform
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = buffer_->lookupTransform(target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, fixed_frame_id_);
            }
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Problem finding transform:\n" <<ex.what());
                return false;
            }

            DIDiffRequest request;
            // request.params.neg_eps = -0.2;
            // request.params.pos_eps = 0.2;
            // request.params.fill_cloud = true;
            // request.params.fill_im = true;
            // request.params.fill_debug = true;

            RCLCPP_INFO_STREAM(node_->get_logger(), "dii calling insertPoints ...");
            // sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            insertPoints(cylindrical_points, image_msg, cam_model_, transform, request); // , 
            // auto pcloud_msg = request.results.point_cloud;
            // pcloud_msg->header = target_header; //image_msg->header;
            // pub_diff_pc_.publish(pcloud_msg);

            // auto depthim_msg = request.results.depth_image;
            // // depthim_msg->header
            // pub_diff_im_.publish(depthim_msg);

            // debug_pub_.publish(request.results.debug);

            return true;
        }

        bool DepthImageInserter::insert(ECWrapper& cylindrical_points, 
                                        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info,
                                        const sensor_msgs::msg::Image::ConstSharedPtr& normals_msg)                                        
        {
            const std_msgs::msg::Header& target_header = cylindrical_points.getHeader();
            const std_msgs::msg::Header& source_header = image_msg->header;

            if(target_header == source_header)
            {
                // ROS_INFO_ONCE("Target and source headers match, using remapping approach");
                // // ROS_DEBUG_STREAM_NAMED("labels", "we are remapping now!"); // not happening it appears
                depth_remapper_.update(cylindrical_points, image_msg, cam_info);
                return true;
            }
            
            if( cam_model_.fromCameraInfo(cam_info) )
            {
                // ROS_DEBUG("Camera info has changed!");
                //If camera info changed, update any precomputed values
                //cam_model_.init();
            }
            else
            {
                // ROS_DEBUG("Camera info has not changed");
            }
            
            //Get transform
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = buffer_->lookupTransform(target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, fixed_frame_id_);
            }
            catch (tf2::TransformException &ex) 
            {
                // ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                return false;
            }

            DIDiffRequest request;
            // request.params.neg_eps = -0.2;
            // request.params.pos_eps = 0.2;
            // request.params.fill_cloud = true;
            // request.params.fill_im = true;
            // request.params.fill_debug = true;

            // sensor_msgs::msg::PointCloud2::SharedPtr pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            insertPoints(cylindrical_points, image_msg, normals_msg, cam_model_, transform, request); // , 
            // auto pcloud_msg = request.results.point_cloud;
            // pcloud_msg->header = target_header; //image_msg->header;
            // pub_diff_pc_.publish(pcloud_msg);

            // auto depthim_msg = request.results.depth_image;
            // // depthim_msg->header
            // pub_diff_im_.publish(depthim_msg);

            // debug_pub_.publish(request.results.debug);

            return true;
        }

        bool DepthImageInserter::insert(ECWrapper& cylindrical_points, 
                                        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info, 
                                        const sensor_msgs::msg::Image::ConstSharedPtr& normals_msg,
                                        const sensor_msgs::msg::Image::ConstSharedPtr& labels_msg)                                        
        {
            const std_msgs::msg::Header& target_header = cylindrical_points.getHeader();
            const std_msgs::msg::Header& source_header = image_msg->header;

            if(target_header == source_header)
            {
                // ROS_INFO_ONCE("Target and source headers match, using remapping approach");
                // // ROS_DEBUG_STREAM_NAMED("labels", "we are remapping now!"); // not happening it appears
                depth_remapper_.update(cylindrical_points, image_msg, cam_info);
                return true;
            }
            
            if( cam_model_.fromCameraInfo(cam_info) )
            {
                // ROS_DEBUG("Camera info has changed!");
                //If camera info changed, update any precomputed values
                //cam_model_.init();
            }
            else
            {
                // ROS_DEBUG("Camera info has not changed");
            }
            
            //Get transform
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = buffer_->lookupTransform(target_header.frame_id, target_header.stamp, 
                                                    source_header.frame_id, source_header.stamp, fixed_frame_id_);
            }
            catch (tf2::TransformException &ex) 
            {
                // ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                return false;
            }

            DIDiffRequest request;
            insertPoints(cylindrical_points, image_msg, normals_msg, labels_msg, cam_model_, transform, request);  

            return true;
        }


    } //end namespace utils
} //end namespace egocylindrical

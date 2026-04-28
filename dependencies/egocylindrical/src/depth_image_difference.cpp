#include <egocylindrical/depth_image_difference.h>

#include <egocylindrical/ecwrapper.h>   //redundant
#include <egocylindrical/depth_image_common.h>  //redundant
#include <egocylindrical/range_image_common.h>
#include <egocylindrical/point_transformer_object.h>

#include <cv_bridge/cv_bridge.h> //redundant
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cstdlib>

#include <tf2/LinearMath/Transform.h>

//These are only needed to get dilatedPoints()
#include <egocylindrical/range_image_core.h>
#include <egocylindrical/range_image_dilator_core.h>
#include <egocylindrical/range_image_core.h>
#include <egocylindrical/range_to_points.h>
#include <egocylindrical/point_cloud_core.h>


namespace tf2
{
    //Both functions copied from tf2/src/buffer_core.cpp; ideally, tf2 would declare them in a header so we could just use them directly

    void transformMsgToTF2(const geometry_msgs::msg::Transform& msg, tf2::Transform& tf2)
    {tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

    /** \brief convert Transform to Transform msg*/
    void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::Transform& msg)
    {
        msg.translation.x = tf2.getOrigin().x();
        msg.translation.y = tf2.getOrigin().y();
        msg.translation.z = tf2.getOrigin().z();
        msg.rotation.x = tf2.getRotation().x();
        msg.rotation.y = tf2.getRotation().y();
        msg.rotation.z = tf2.getRotation().z();
        msg.rotation.w = tf2.getRotation().w();
    }
}


namespace egocylindrical
{
    namespace utils
    {
        

        geometry_msgs::msg::TransformStamped getInverse(const geometry_msgs::msg::TransformStamped& transform_msg)
        {
            tf2::Transform transform;
            transformMsgToTF2(transform_msg.transform, transform);
            geometry_msgs::msg::TransformStamped inverted_transform_msg;
            tf2::transformTF2ToMsg(transform.inverse(), inverted_transform_msg.transform);
            inverted_transform_msg.header.stamp = transform_msg.header.stamp;
            inverted_transform_msg.header.frame_id = transform_msg.child_frame_id;
            inverted_transform_msg.child_frame_id = transform_msg.header.frame_id;

            return inverted_transform_msg;
        }

        visualization_msgs::msg::Marker getECPointMarker(const utils::ECWrapper& ec_points, std::string ns="points", float scale=0.025, float r=1, float g=1, float b=1, float a=1)
        {
            visualization_msgs::msg::Marker m;
            m.type = visualization_msgs::msg::Marker::POINTS;
            m.ns = ns;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.id = 0;
            m.header = ec_points.getHeader();
            m.pose.orientation.w = 1;
            m.scale.x = scale;
            m.scale.y = scale;
            m.color.r = r;
            m.color.g = g;
            m.color.b = b;
            m.color.a = a;

            const float* x = ec_points.getX();
            const float* y = ec_points.getY();
            const float* z = ec_points.getZ();

            for(int i = 0; i < ec_points.getNumPts(); ++i)
            {
                geometry_msgs::msg::Point p;
                p.x = x[i];
                p.y = y[i];
                p.z = z[i];
                if(p.x==p.x)
                {
                    m.points.push_back(p);
                }
            }

            return m;
        }

        //immediate insert
        template <typename T>
        void insertPoints6_impl(const utils::ECWrapper& original_cylindrical_points, const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg_ptr, const CleanCameraModel& cam_model, const geometry_msgs::msg::TransformStamped transform, DIDiffRequest& request)
        {
            auto getDilatedPoints = [&request](const utils::ECWrapper& cyl_points)    //  utils::ECWrapperPtr
            {
                // ros::WallTimerange_// start = ros::WallTime::now();
                sensor_msgs::msg::Image::SharedPtr range_image_ptr = utils::getRawRangeImageMsg(cyl_points, 1);
                request.results.debug.range_image = range_image_ptr;

                // ros::WallTimepc_// start = ros::WallTime::now();
                sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_ptr = utils::generate_point_cloud(cyl_points);
                request.results.debug.point_cloud = point_cloud_ptr;

                // ros::WallTimedilate_// start = ros::WallTime::now();
                sensor_msgs::msg::Image::SharedPtr dilated_image_ptr = dilateImage(range_image_ptr);
                request.results.debug.dilated_range_image = dilated_image_ptr;

                // ros::WallTimewrapper_// start = ros::WallTime::now();
                ECMsgConstPtr info = cyl_points.getEgoCylinderInfoMsg();
                utils::ECWrapperPtr ec_pts = utils::range_image_to_wrapper(info, dilated_image_ptr, nullptr);

                // ros::WallTimedilated_pc_// start = ros::WallTime::now();
                sensor_msgs::msg::PointCloud2::SharedPtr dilated_point_cloud_ptr = utils::generate_point_cloud(*ec_pts);
                request.results.debug.dilated_point_cloud = dilated_point_cloud_ptr;

                // ros::WallTimeend_time = ros::WallTime::now();

                if(request.params.fill_debug)
                {
                    visualization_msgs::msg::MarkerArray::SharedPtr& marker_array = request.results.debug.marker_array;
                    marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

                    visualization_msgs::msg::Marker m_cyl = getECPointMarker(cyl_points, "ec_points", 0.025, 0.6, 0.1, 0.3, 1);
                    marker_array->markers.push_back(m_cyl);

                    visualization_msgs::msg::Marker m_dil_cyl = getECPointMarker(*ec_pts, "dilated_ec_points", 0.025, 0.2, 0.6, 0.2, 1);
                    m_cyl.ns = "dilated_ec_points";
                    m_cyl.color.r = 0.2;
                    m_cyl.color.g = 0.6;
                    m_cyl.color.b = 0.2;
                    m_cyl.color.a = 1;
                    marker_array->markers.push_back(m_dil_cyl);

                }


                // ROS_DEBUG_STREAM_NAMED("depth_diff.timing", "Generating range image took " <<  (pc_start - range_start).toSec() * 1e3 << "ms");
                // ROS_DEBUG_STREAM_NAMED("depth_diff.timing", "Generating point cloud took " <<  (dilate_start - pc_start).toSec() * 1e3 << "ms");
                // ROS_DEBUG_STREAM_NAMED("depth_diff.timing", "Dilating range image took " <<  (wrapper_start - dilate_start).toSec() * 1e3 << "ms");
                // ROS_DEBUG_STREAM_NAMED("depth_diff.timing", "Converting dilated range image to wrapper took " <<  (dilated_pc_start - wrapper_start).toSec() * 1e3 << "ms");
                // ROS_DEBUG_STREAM_NAMED("depth_diff.timing", "Converting dilated wrapper to point cloud took " <<  (end_time - dilated_pc_start).toSec() * 1e3 << "ms");
                // ROS_INFO_STREAM_NAMED("depth_diff.timing", "Total dilation process took " <<  (end_time - range_start).toSec() * 1e3 << "ms");

                return ec_pts;
            };

            utils::ECWrapper::Ptr cylindrical_points_ptr = getDilatedPoints(original_cylindrical_points);
//             auto& cylindrical_points = original_cylindrical_points;
            auto& cylindrical_points = *cylindrical_points_ptr;


            auto neg_eps = request.params.neg_eps;
            auto pos_eps = request.params.pos_eps;
            
            bool fill_cloud = request.params.fill_cloud;
            bool fill_im = request.params.fill_im;
            bool fill_debug = request.params.fill_debug;

            sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg = request.results.point_cloud;
            sensor_msgs::msg::Image::SharedPtr& gen_im_msg = request.results.depth_image;
            const sensor_msgs::msg::Image& image_msg = *image_msg_ptr;

            cv::Mat dilated_range_image = cv_bridge::toCvCopy(request.results.debug.dilated_range_image)->image;

            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();

            PointTransformerObject inverse_point_transformer(getInverse(transform));

            // T* gen_im_data;
            // if(fill_im)
            // {
            //     const int num_pixels = image_width * image_height;
            //     gen_im_msg = std::make_shared<sensor_msgs::msg::Image>();
            //     gen_im_msg->data.resize(sizeof(T) * num_pixels);
            //     gen_im_data = (T*) gen_im_msg->data.data();
            // }

            cv::Mat gen_im_mat;//(image.rows(), image.cols(), image.type());
            if(fill_im)
            {
                gen_im_mat = cv::Mat(image.rows, image.cols, image.type(), dNaN);
            }

            visualization_msgs::msg::Marker novel_pc_marker;
            if(fill_debug)
            {
                request.results.debug.depth_image = std::make_shared<sensor_msgs::msg::Image>(*image_msg_ptr); //Any reason not to just use image_msg_ptr directly here?

                novel_pc_marker.type = visualization_msgs::msg::Marker::POINTS;
                novel_pc_marker.ns = "novel_points";
                novel_pc_marker.action = visualization_msgs::msg::Marker::ADD;
                novel_pc_marker.id = 0;
                novel_pc_marker.header = transform.header;  //.frame_id = transform.child_frame_id;
                // novel_pc_marker.header.stamp = transform.header.stamp;
                novel_pc_marker.color.a = 1;
                novel_pc_marker.color.g = 1;
                novel_pc_marker.color.b = 1;
                novel_pc_marker.scale.x = 0.025;
                novel_pc_marker.scale.y = 0.025;
                novel_pc_marker.pose.orientation.w = 1;

                // request.results.debug.marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
            }
            

            float* pc_data=nullptr;
            if(fill_cloud)
            {
                const int num_pixels = image_width * image_height;
                pcl::PointCloud<pcl::PointXYZ> pcloud;
                pcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(pcloud, *pcloud_msg);
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_pixels);
                pcloud_msg->is_dense = false;  //should be false, but seems to work with true
                pc_data = (float*) pcloud_msg->data.data();
            }

            cv::Mat range_mat = cv_bridge::toCvShare(request.results.debug.range_image)->image;
            cv::Mat non_nans_mat = (range_mat == range_mat);
            int num_filled = cv::countNonZero(non_nans_mat);
            // ROS_INFO_STREAM("Num range image pixel filled: " << num_filled);

            int pc_counter = 0;
            // ros::WallTimereprojection_// start = ros::WallTime::now();
            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {                       
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    // cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                    // cv::Point3f world_pnt = ray * (((float) depth)/scale);
                    // cv::Point3f transformed_ray = point_transformer.transform(ray);
                    
                    

                    T depth = image.at<T>(i,j);
                    bool vr = !std::isnan(depth);
                    if(!vr)
                    {
                        // depth = 1;
                    }
                    

                    // if(depth>0)  //Only insert actual points (works for both float and uint16)
                    {                        
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (((float) depth)/scale);
                        cv::Point3f transformed_ray = point_transformer.transform(ray);

                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        bool save_point = false;
                        
                        if(cyl_idx >= 0  && cyl_idx < max_ind)
                        {
                            cv::Point img_inds = cylindrical_points.project3dToPixel(transformed_ray);

                            // float prev_range_f = (prev_range_uint > 0) ? prev_range_uint /1000.0 : dNaN;

                            float prev_range_f;
                            if(dilated_range_image.depth() == CV_32FC1)
                            {
                                // float v = dilated_range_image.at<float>(img_inds);
                                // v = (RangeVals<float>::)
                                prev_range_f = dilated_range_image.at<float>(img_inds)/RangeVals<float>::scale();
                            }
                            else if(dilated_range_image.depth() == CV_16UC1)
                            {
                                uint16_t v = dilated_range_image.at<uint16_t>(img_inds);
                                prev_range_f = (RangeVals<uint16_t>::is_valid(v)) ? float(v)/RangeVals<uint16_t>::scale() : dNaN;
                            }

                            if(num_filled > 0 && prev_range_f != prev_range_f)
                            {
                                // ROS_DEBUG_STREAM_NAMED("weird_nans", "depth im coords with unexpected NaN: " << pt);
                            }

                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            float prev_range_sq = worldToRangeSquared(prev_point);
                            float prev_range = std::sqrt(prev_range_sq);
                            bool vpr = !std::isnan(prev_range);

                            float range_sq = worldToRangeSquared(transformed_pnt);
                            float range = std::sqrt(range_sq);
                            
                            {
                                if(!std::isnan(prev_range) && !std::isnan(prev_range_f))
                                {
                                    float pr_diff = prev_range_f - prev_range;
                                }
                            }

                            if(vr && vpr)
                            {
                                float diff = range - prev_range;
                                if(diff > pos_eps || diff < neg_eps)
                                {
                                    // save_point = true;
                                }
                            }
                            else if(vr)
                            {
                                save_point = true;
                            }
                            else //if(vpr)
                            {
                                // save_point = false;
                            }

                            if (fill_im)
                            {
                                auto pp = inverse_point_transformer.transform(prev_point);
                                // gen_im_data[i + j*image_width] = (T)(scale * pp.z);
                                gen_im_mat.at<T>(i,j) = (T)(scale * pp.z);
                            }

                            if(save_point)
                            {
                                //TODO: Move this logic to separate class?
                                if(fill_cloud)
                                {
                                    pc_data[4*pc_counter] = transformed_pnt.x;
                                    pc_data[4*pc_counter+1] = transformed_pnt.y;
                                    pc_data[4*pc_counter+2] = transformed_pnt.z;
                                    pc_data[4*pc_counter+3] = 1;  //Not necessary, only include if improves performance
                                
                                    ++pc_counter;
                                }

                                if(fill_debug)
                                {
                                    geometry_msgs::msg::Point p;
                                    p.x = transformed_pnt.x;
                                    p.y = transformed_pnt.y;
                                    p.z = transformed_pnt.z;
                                    novel_pc_marker.points.push_back(p);
                                }
                            }
                        }
                        else if(false)
                        {
                            cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                        
                            if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts())
                            {
                                float can_depth = worldToCanDepth(transformed_pnt);
                                
                                cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                                
                                float prev_can_depth = worldToCanDepth(prev_point);
                                
                                bool vd = !std::isnan(can_depth);
                                bool vpd = !std::isnan(prev_can_depth);

                                if(vd && vpd)
                                {
                                    float diff = can_depth - prev_can_depth;
                                    if(diff > pos_eps || diff < neg_eps)
                                    {
                                        save_point = true;
                                    }
                                }
                                else if(vd)
                                {
                                    save_point = true;
                                }
                                else //if(vpr)
                                {
                                    //save_point = false;
                                }
                            }
                            //TODO: fill in gen_im_data here
                        }

                        // if(save_point)
                        // {
                        //     //TODO: Move this logic to separate class?
                        //     if(fill_cloud)
                        //     {
                        //         pc_data[4*j] = transformed_pnt.x;
                        //         pc_data[4*j+1] = transformed_pnt.y;
                        //         pc_data[4*j+2] = transformed_pnt.z;
                        //         pc_data[4*j+3] = 1;  //Not necessary, only include if improves performance
                            
                        //         ++pc_counter;
                        //     }
                        // }
                    }
                    
                }
            }
            
            // ROS_INFO_STREAM("Time to reproject range image to depth image: " << (ros::WallTime::now() - reprojection_start).toSec()*1000 << "ms");

            if(fill_cloud)
            {
                //TODO: if no points added to point cloud, don't bother publishing
                pcloud_msg->width = pc_counter;
                pcloud_msg->height = 1;
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ)*pc_counter);
                pcloud_msg->row_step = static_cast<uint32_t> (sizeof (pcl::PointXYZ) * pcloud_msg->width);
            }

            if(fill_im)
            {
                cv_bridge::CvImage cvim;
                cvim.image = gen_im_mat;
                cvim.header = image_msg.header;
                cvim.encoding = image_msg.encoding;
                gen_im_msg = cvim.toImageMsg();
                // const int num_pixels = image_width * image_height;
                // gen_im_msg = std::make_shared<sensor_msgs::msg::Image>();
                // gen_im_msg->data.resize(sizeof(T) * num_pixels);
                // gen_im_data = (T*) gen_im_msg->data.data();

                if(fill_debug)
                {
                    cv::Mat depth_diff_im = gen_im_mat - image;
                    cvim.image = depth_diff_im;
                    request.results.debug.depth_diff_image = cvim.toImageMsg();
                    request.results.debug.reproj_depth_image = gen_im_msg;
                }
            }

            if(fill_debug)
            {
                if(novel_pc_marker.points.size()==0)
                {
                    novel_pc_marker.action = visualization_msgs::msg::Marker::DELETE;
                }
                request.results.debug.marker_array->markers.push_back(novel_pc_marker);
            }
        }


        void insertPoints6(const utils::ECWrapper& cylindrical_points, const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const CleanCameraModel& cam_model, const geometry_msgs::msg::TransformStamped transform, DIDiffRequest& request)
        {
            if(image.depth() == CV_32FC1)
            {
                insertPoints6_impl<float>(cylindrical_points, image, image_msg, cam_model, transform, request);
            }
            else if (image.depth() == CV_16UC1)
            {
                insertPoints6_impl<uint16_t>(cylindrical_points, image, image_msg, cam_model, transform, request);
            }
            // ROS_DEBUG_STREAM("Finished inserting points and stuff");
        }

        // template <uint16_t>
        // void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::msg::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg);

        // template <typename float>
        // void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::msg::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::msg::PointCloud2::SharedPtr& pcloud_msg);

    } //end namespace utils
} //end namespace egocylindrical

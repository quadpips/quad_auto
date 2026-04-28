#include <egocylindrical/point_transformer.h>
#include <egocylindrical/point_transformer_object.h>
#include <egocylindrical/ecwrapper.h>

#include <rclcpp/rclcpp.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <image_transport/image_transport.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <omp.h>

#include <geometry_msgs/msg/transform_stamped.hpp>


namespace egocylindrical
{
    
    namespace utils
    {

        //
        inline
        void transform_impl(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const PointTransformerObject& pto, int num_threads)
        {            
            const int max_ind = new_points.getCols();
            const int num_pts = points.getNumPts();
            
            // Points
            const float* x = (const float*)__builtin_assume_aligned(points.getX(), __BIGGEST_ALIGNMENT__);
            const float* y = (const float*)points.getY();
            const float* z = (const float*)points.getZ();

            // Normals
            const float* norm_x = (const float*)__builtin_assume_aligned(points.getNormalX(), __BIGGEST_ALIGNMENT__);
            const float* norm_y = (const float*)points.getNormalY();
            const float* norm_z = (const float*)points.getNormalZ();

            // Labels
            const uint8_t* labels = (const uint8_t*)points.getLabels();

            // New points
            float* x_n = (float*)__builtin_assume_aligned(transformed_points.getX(), __BIGGEST_ALIGNMENT__);
            float* y_n = (float*)transformed_points.getY();
            float* z_n = (float*)transformed_points.getZ();
            
            // New normals
            float* norm_x_n = (float*)__builtin_assume_aligned(transformed_points.getNormalX(), __BIGGEST_ALIGNMENT__);
            float* norm_y_n = (float*)transformed_points.getNormalY();
            float* norm_z_n = (float*)transformed_points.getNormalZ();

            // New labels
            uint8_t* labels_n = (uint8_t*)transformed_points.getLabels();

            float* ranges = transformed_points.getRanges();
            
            
            //#ifndef PIPS_ON_ARM
            
            int32_t* inds = (int32_t*)__builtin_assume_aligned(transformed_points.getInds(), __BIGGEST_ALIGNMENT__);
            //#endif

            
            if (omp_get_dynamic())
                omp_set_dynamic(0);
            omp_set_nested(1);
            int omp_p = omp_get_max_threads();
            
            omp_p = std::min(omp_p-1, num_threads);
            
            
            #pragma omp parallel num_threads(omp_p)
            {
                
                #pragma omp single nowait
                {
                    if(omp_in_parallel())
                    {
                        // ROS_DEBUG_STREAM("Parallel region with " << omp_get_num_threads() << " threads");
                    }
                }
                
                
                //#pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
                #pragma omp for simd schedule(static) aligned(x:__BIGGEST_ALIGNMENT__) aligned(x_n:__BIGGEST_ALIGNMENT__) aligned(ranges:__BIGGEST_ALIGNMENT__) aligned(inds:__BIGGEST_ALIGNMENT__)
                for(int p = 0; p < num_pts; ++p)
                {
                    float x_p = x[p];
                    float y_p = y[p];
                    float z_p = z[p];
                    cv::Point3f world_pnt(x_p, y_p, z_p);

                    pto.transform(x_p, y_p, z_p, x_n[p], y_n[p], z_n[p]);  

                    // cv::Point3f transformed_pnt(x_n[p], y_n[p], z_n[p]);

                    float nx_p = norm_x[p];
                    float ny_p = norm_y[p];
                    float nz_p = norm_z[p];
                    cv::Point3f world_norm(nx_p, ny_p, nz_p);

                    pto.rotate(nx_p, ny_p, nz_p, norm_x_n[p], norm_y_n[p], norm_z_n[p]);

                    // cv::Point3f world_pnt_plus_norm = world_pnt + world_norm;
                    // cv::Point3f transformed_pnt_plus_norm = pto.transform(world_pnt_plus_norm); // transform between prior timestep and current timestep (or something, not so sure)

                    // cv::Point3f transformed_norm = transformed_pnt_plus_norm - transformed_pnt;
                    // cv::Point3f re_normalized_norm = transformed_norm / cv::norm(transformed_norm);

                    // pto.rotate(nx_p, ny_p, nz_p, norm_x_n[p], norm_y_n[p], norm_z_n[p]);   
                    // norm_x_n[p] = re_normalized_norm.x;
                    // norm_y_n[p] = re_normalized_norm.y;
                    // norm_z_n[p] = re_normalized_norm.z;      
                                    
                    float range_squared= worldToRangeSquared(x_n[p],z_n[p]);
                
                    int idx = -1;
                    {
                      int tidx = new_points.worldToCylindricalIdx(x_n[p],y_n[p],z_n[p]);
                      
                      if(tidx < max_ind)
                      {
                        idx = tidx;
                      }
                    }
                    
                    inds[p] = idx;
                    
                    ranges[p] = range_squared;

                    labels_n[p] = labels[p];
                    
                }
                
            }
        }
        
        
        
        // TODO: This functionality could be moved into a tf2_ros implementation, though it wouldn't be able to populate the 'inds'
        void transformPoints(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const geometry_msgs::msg::TransformStamped& trans, int num_threads)
        {
              PointTransformerObject pto(trans);
            
          //  if(points.isLocked())
          //  {
                // ROS_DEBUG("Init transformed ECWrapper");
                transformed_points.init(points);    //This ensures that 'transformed_points' is big enough
                transform_impl(points, transformed_points, new_points, pto, num_threads);
        /*
            }
            else
            {
                // ROS_INFO("In place");
                transformed_points.useStorageFrom(points);
                transform_impl(points, transformed_points, new_points, rotationArray, translationArray);
            }
            */
            
        }
        
    }
    
}

#ifndef EGOCYLINDRICAL_ADD_POINTS_H
#define EGOCYLINDRICAL_ADD_POINTS_H

#include <egocylindrical/ecwrapper.h>

// #include <ros/console.h>
#include <opencv2/core.hpp>

namespace egocylindrical
{

namespace utils
{

    
    inline
    void addPoints(utils::ECWrapper& cylindrical_history, const utils::ECWrapper& new_points, bool overwrite)
    {
        
        float* x = cylindrical_history.getX();
        float* y = cylindrical_history.getY();
        float* z = cylindrical_history.getZ();
        float* x_norms = cylindrical_history.getNormalX();
        float* y_norms = cylindrical_history.getNormalY();
        float* z_norms = cylindrical_history.getNormalZ();
        uint8_t* labels = cylindrical_history.getLabels();

        const float* n_x = new_points.getX();
        const float* n_y = new_points.getY();
        const float* n_z = new_points.getZ();
        const float* n_x_norms = new_points.getNormalX();
        const float* n_y_norms = new_points.getNormalY();
        const float* n_z_norms = new_points.getNormalZ();
        const uint8_t* n_labels = new_points.getLabels();
        
        const float* ranges = new_points.getRanges();
        const int32_t* inds = new_points.getInds();
        
        const bool use_egocan = (new_points.getParams().can_width > 0);

        // ROS_DEBUG("Relocated the propagated image");

        // ROS_DEBUG_STREAM_NAMED("labels", "new_points.getNumPts(): " << new_points.getNumPts());
        // ROS_DEBUG_STREAM_NAMED("labels", "cylindrical_history.getNumPts(): " << cylindrical_history.getNumPts());

        //#pragma omp parallel for
        for(int i = 0; i < new_points.getNumPts(); ++i)
        {
            
            int idx=-1;
            idx= inds[i];
            
            if (idx >=0)
            {
                float depth = ranges[i];
                
                cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                
                float prev_depth = worldToRangeSquared(prev_point);
                
                if (!(prev_depth <= depth)) // if new depth is less than old depth, then update
                {   
                    /*TODO: Check if this gets compiled out or not. If not, remove this object, 
                     * or perhaps use basic templated custom point class to combine benefits of
                     * readability and efficiency
                     */
                    cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
                    cv::Point3f world_norm(n_x_norms[i],n_y_norms[i],n_z_norms[i]);
                    
                    x[idx] = world_pnt.x;
                    y[idx] = world_pnt.y;
                    z[idx] = world_pnt.z;
                    x_norms[idx] = world_norm.x;
                    y_norms[idx] = world_norm.y;
                    z_norms[idx] = world_norm.z;
                    labels[idx] = n_labels[i]; // fine at lab, overwrite label as well
                    // // ROS_DEBUG_STREAM_NAMED("labels", "labels[" << idx << "]:" << std::hex << (uint16_t) labels[idx]);                     

                    // if (n_labels[i] == n_labels[i])
                    // {
                    //     // // ROS_INFO_STREAM_NAMED("labels", "labels[" << idx << "]: " << labels[idx] << " (1)");
                    //     // // ROS_INFO_STREAM_NAMED("labels", "n_labels[" << i << "]: " << n_labels[i] << " (1)");
                    //     // labels[idx] = n_labels[i]; // overwrite label as well
                    // } else
                    // {
                    //     // ROS_INFO_STREAM_NAMED("labels", "n_labels[" << i << "] is NaN (1)"); 
                    // }
                    // // // ROS_INFO_STREAM_NAMED("update", "labels[idx]: " << labels[idx]);
                    // // ROS_INFO_STREAM_NAMED("update", "i: " << i << ", idx: " << idx << ", numPts: " << new_points.getNumPts());
                }
                
            }
            else if (use_egocan)
            {
                if (n_x[i] == n_x[i])  //Skip NaNs
                {
                    cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
                    cv::Point3f world_norm(n_x_norms[i],n_y_norms[i],n_z_norms[i]);
                  
                    idx = new_points.worldToCanIdx(world_pnt);
                    
                    if (idx >=0 && idx < new_points.getNumPts())
                    {
                        float depth = worldToCanDepth(world_pnt);
                          
                        cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                        
                        float prev_depth = worldToCanDepth(prev_point);
                        
                        if (!(prev_depth <= depth)) //overwrite || 
                        {   
                            
                            x[idx] = world_pnt.x;
                            y[idx] = world_pnt.y;
                            z[idx] = world_pnt.z;
                            x_norms[idx] = world_norm.x;
                            y_norms[idx] = world_norm.y;
                            z_norms[idx] = world_norm.z;
                            labels[idx] =  n_labels[i]; // overwrite label as well
                            // // ROS_DEBUG_STREAM_NAMED("labels", "labels[" << idx << "]:" << std::hex << (uint16_t) labels[idx]);                     

                            // if (n_labels[i] == n_labels[i])
                            // {
                            //     // // ROS_INFO_STREAM_NAMED("labels", "labels[" << idx << "]: " << labels[idx] << " (2)");
                            //     // // ROS_INFO_STREAM_NAMED("labels", "n_labels[" << i << "]: " << n_labels[i] << " (1)");
                            //     // labels[idx] =  n_labels[i]; // overwrite label as well
                            // } else
                            // {
                            //     // ROS_INFO_STREAM_NAMED("labels", "n_labels[" << i << "] is NaN (2)"); 
                            // }
                        }
                    }
                    else
                    {
                        // ROS_WARN_STREAM("Invalid index [" << idx << "] for point (" << world_pnt.x << "," << world_pnt.y << "," << world_pnt.z << ")");
                    }
                  
                }
            }
            
            
            
        }
    
        
        
    }
    
    
}

}

#endif  //EGOCYLINDRICAL_ADD_POINTS_H

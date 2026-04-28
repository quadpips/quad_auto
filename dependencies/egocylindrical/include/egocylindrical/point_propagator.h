#ifndef EGOCYLINDRICAL_POINT_PROPAGATOR_H
#define EGOCYLINDRICAL_POINT_PROPAGATOR_H

#include <egocylindrical/ecwrapper.h>
#include <tf2_ros/buffer.h>


namespace egocylindrical
{
/*    
    utils::ECParams getParams(const egocylindrical::PropagatorConfig &config)
    {
      utils::ECParams params;
      params.height = config.height;
      params.width = config.width;
      params.vfov = config.vfov;
      params.can_width = config.can_width;
      params.v_offset = config.v_offset;
      params.cyl_radius = config.cyl_radius;
      return params;
    }
    
    namespace utils
    {
      utils::ECWrapperPtr getECWrapper(const egocylindrical::PropagatorConfig &config, bool allocate_arrays=false)
      {
        return utils::getECWrapper(getParams(config), allocate_arrays);
      }
    }*/

namespace utils
{

    
    //Needs fixed_frame, buffer, transformed pts storage
    //To transform, just needs old points, desired frame_id (could actually move coordinate frame helper into this class), and new points storage
    class PointPropagator
    {
    public:
      PointPropagator(std::shared_ptr<tf2_ros::Buffer> buffer);
      
      bool init(std::string fixed_frame_id);
      
      void transform(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, const std_msgs::msg::Header& new_header);
      
    protected:
      std::string fixed_frame_id_;
      // tf2_ros::Buffer& buffer_;
      std::shared_ptr<tf2_ros::Buffer> buffer_;

      utils::ECWrapper::Ptr transformed_pts_;
      int num_threads_ = 1; // Default to single-threaded, can be set externally
    };
    
    
    //Only perform clearing if measurement frame_id has same origin as target frame
} //end namespace utils
    
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_POINT_PROPAGATOR_H

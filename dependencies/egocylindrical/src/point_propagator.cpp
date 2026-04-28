#include <egocylindrical/point_propagator.h>
#include <egocylindrical/point_transformer.h>
#include <egocylindrical/add_points.h>

namespace egocylindrical
{

namespace utils
{
    PointPropagator::PointPropagator(std::shared_ptr<tf2_ros::Buffer> buffer_):
      buffer_(buffer_)
      {}
      
    bool PointPropagator::init(std::string fixed_frame_id)
    {
      fixed_frame_id_ = fixed_frame_id;
      transformed_pts_ = utils::getECWrapper(utils::ECParams(), true);
      return true;
    }
    
    void PointPropagator::transform(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, const std_msgs::msg::Header& new_header)
    {
      // ros::WallTime // start = ros::WallTime::now();
      
      std_msgs::msg::Header old_header = old_pnts.getHeader();
      
      new_pnts.setHeader(new_header);
      
      // ROS_DEBUG("Getting Transformation details");
      geometry_msgs::msg::TransformStamped trans = buffer_->lookupTransform(new_header.frame_id, new_header.stamp,
                                                                            old_header.frame_id, old_header.stamp,
                                                                            fixed_frame_id_);
      
      // ROS_DEBUG_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
              
      
      // start = ros::WallTime::now();
      // NOTE: If there is a benefit to transforming points in place when possible, add that logic here
      utils::transformPoints(old_pnts, *transformed_pts_, new_pnts, trans, num_threads_);
      // ROS_DEBUG_STREAM_NAMED("timing", "Transforming points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
      // start = ros::WallTime::now();
      utils::addPoints(new_pnts, *transformed_pts_, false);
      // ROS_DEBUG_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
    }
    
}

}

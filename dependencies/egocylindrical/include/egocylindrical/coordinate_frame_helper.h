#ifndef EGOCYLINDRICAL_COORDINATE_FRAME_HELPER_H
#define EGOCYLINDRICAL_COORDINATE_FRAME_HELPER_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <ros/node_handle.h>
// #include <ros/console.h>

#include "builtin_interfaces/msg/time.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>

namespace egocylindrical
{
    namespace utils 
    {
        
        struct CoordinateFrameDefinition
        {
            std::string origin_fixed_frame_id, orientation_fixed_frame_id;
            geometry_msgs::msg::PoseStamped pose;
        };
        
        class CoordinateFrameHelper
        {
        protected:
            // tf2_ros::Buffer& buffer_;
            std::shared_ptr<tf2_ros::Buffer> buffer_;

            std::string fixed_frame_id_;
            // ros::NodeHandle pnh_;
            rclcpp::Node::SharedPtr node_;

            // tf2_ros::TransformBroadcaster tf_br_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

            std::string target_frame_id_;
            geometry_msgs::msg::TransformStamped offset_transform_, ecs_, ecc_;
            CoordinateFrameDefinition cfd_;
            std::string tf_prefix_;
            //ros::Time last_update_;
            std_msgs::msg::Header target_header_;
            bool new_cfd_;

            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
            // rclcpp::Duration nano_second;
            builtin_interfaces::msg::Duration nano_second;

            std::mutex cfd_mutex_;
            using Lock = const std::lock_guard<std::mutex>;
            
        public:

            CoordinateFrameHelper(std::shared_ptr<tf2_ros::Buffer> buffer, rclcpp::Node::SharedPtr node):
                buffer_(buffer),
                node_(node),
                // tf_br_(),
                tf_prefix_(""),
                new_cfd_(false)
                // nano_second(0,1)
            {
                tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
                nano_second.sec = 0;
                nano_second.nanosec = 1;
            }
            
            bool init()
            {
                //TODO: use pips::param_utils
                bool status = node_->get_parameter("fixed_frame_id", fixed_frame_id_);
                
                CoordinateFrameDefinition cfd;
                //Is origin_fixed_frame_id even necessary? Seems like egocan_stabilized's origin must always be at the origin of the camera optical frame
                status &= node_->get_parameter("origin_fixed_frame_id", cfd.origin_fixed_frame_id);
                status &= node_->get_parameter("orientation_fixed_frame_id", cfd.orientation_fixed_frame_id);
                
                cfd.pose.header.frame_id = cfd.orientation_fixed_frame_id;
                cfd.pose.pose.orientation.w=1;
                
                if (status)
                {
                    RCLCPP_INFO_STREAM(node_->get_logger(), "Found all necessary coordinate frame parameters!");
                    updateDefinition(cfd);
                    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("desired_pose", 2, std::bind(&CoordinateFrameHelper::desPoseCB, this, std::placeholders::_1));
                }
                else
                {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "Did not find all necessary coordinate frame parameters! Values are: origin_fixed_frame_id=" << cfd.origin_fixed_frame_id << ", orientation_fixed_frame_id=" << cfd.orientation_fixed_frame_id << ", fixed_frame_id=" << fixed_frame_id_ << ". Defaulting to locking egocan to camera frame.");
                }

                return status;
            }
            
            bool updateTransforms(std_msgs::msg::Header sensor_header)
            {
                Lock lock(cfd_mutex_);

                // RCLCPP_INFO_STREAM(node_->get_logger(), "[updateTransforms] sensor_header: " << sensor_header.frame_id << " at " << sensor_header.stamp.sec << "." << sensor_header.stamp.nanosec);

                //TODO: Lock a recursive mutex?
                //TODO: possibly only specify orientation fixed frame, since should really be using same origin as the camera regardless
                if (cfd_.orientation_fixed_frame_id=="" || cfd_.origin_fixed_frame_id=="")
                {
                    RCLCPP_WARN_ONCE(node_->get_logger(), "[updateTransforms] Frame not specified, using camera frame");//, using camera frame");
                    //cfd_.origin_fixed_frame_id = cfd_.origin_fixed_frame_id= sensor_header.frame_id;
                    target_header_ = sensor_header;
                    return true;
                }
                else if (cfd_.orientation_fixed_frame_id==cfd_.origin_fixed_frame_id && cfd_.origin_fixed_frame_id==sensor_header.frame_id)
                {
                    RCLCPP_WARN_ONCE(node_->get_logger(), "[updateTransforms] Desired frames match camera frame, using camera frame");//, using camera frame");
                    //cfd_.origin_fixed_frame_id = cfd_.origin_fixed_frame_id= sensor_header.frame_id;
                    target_header_ = sensor_header;
                    return true;
                }
              
                bool status = updateECSTransform(sensor_header.stamp);
                status &= updateOffsetTransform(sensor_header.stamp);
                status &= updateECCTransform(sensor_header.stamp);
                
                if(status)
                {
                    target_header_.frame_id = getECCFrameId();
                    target_header_.stamp = sensor_header.stamp;
                }
                return status;
            }
            
            void updateDefinition(CoordinateFrameDefinition cfd)
            {
                {
                    Lock lock(cfd_mutex_);
                    
                    cfd_ = cfd;
                    new_cfd_ = true;
                }
                
                
//                 if(!updateECSTransform(ros::Time()))
//                 {
//                     return;
//                 }
//                 if(!updateOffsetTransform(ros::Time()))
//                 {
//                     return;
//                 }

                // RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully updated offset definition!");
                return;
            }
            
            std_msgs::msg::Header getTargetHeader()
            {
                return target_header_;
            }
            
            
        protected:
            std::string getECSFrameId()
            {
                return tf_prefix_ + "egocan_stabilized";
            }
            
            std::string getECFrameId()
            {
                return tf_prefix_ + "egocan";
            }
            
            std::string getECCFrameId()
            {
                return tf_prefix_ + "egocan_camera";
            }
            
            void desPoseCB(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose)
            {
                cfd_.pose = *pose;
                updateDefinition(cfd_);
            }

            bool updateECSTransform(builtin_interfaces::msg::Time stamp)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "[updateECSTransform]");

                // TODO: Lock a mutex
              
                // if(cfd_.orientation_fixed_frame_id==cfd_.origin_fixed_frame_id)
                // {
                //      // RCLCPP_WARN_STREAM(node_->get_logger(), "[updateECSTransform] Not publishing redundant transform! " << stamp);
                //     return true;
                // }

                //If we've already computed it, don't do it again
                if (stamp == ecs_.header.stamp && stamp != node_->get_clock()->now())
                {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "[updateECSTransform] Not publishing redundant transform! " << stamp.sec << "." << stamp.nanosec);
                    return true;
                }
                 
                // RCLCPP_INFO_STREAM(node_->get_logger(), "Trying to transform from " << cfd_.origin_fixed_frame_id << " to " << cfd_.orientation_fixed_frame_id << " at " << stamp.sec << "." << stamp.nanosec);
                try
                {
                    ecs_ = buffer_->lookupTransform(cfd_.orientation_fixed_frame_id, stamp, 
                                                    cfd_.origin_fixed_frame_id, stamp, fixed_frame_id_);

                    ecs_.transform.rotation = geometry_msgs::msg::Quaternion();
                    ecs_.transform.rotation.w=1;
                    ecs_.child_frame_id = tf_prefix_ + "egocan_stabilized";
                }
                catch (tf2::TransformException &ex) 
                {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "Problem finding transform:\n" <<ex.what());
                    return false;
                }

                // RCLCPP_DEBUG_STREAM(node_->get_logger(), "[updateECSTransform] Updated transform! " << stamp.sec << "." << stamp.nanosec);

                ecs_.header.stamp.nanosec += nano_second.nanosec;
                buffer_->setTransform(ecs_, "coordinate_frame_helper", false);
                ecs_.header.stamp.nanosec -= nano_second.nanosec;
                tf_br_->sendTransform(ecs_);
                
                return true;
            }

            bool updateECCTransform(builtin_interfaces::msg::Time stamp)
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "[updateECCTransform]");

                if (stamp == ecc_.header.stamp && stamp != builtin_interfaces::msg::Time())
                {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "[updateECCTransform] Not publishing redundant transform! " << stamp.sec << "." << stamp.nanosec);
                    return true;
                }
                
                //geometry_msgs::msg::TransformStamped ecc;
                ecc_.header.stamp = stamp;
                ecc_.header.frame_id = getECFrameId();
                ecc_.child_frame_id = getECCFrameId();
                auto& q = ecc_.transform.rotation;
                q.x=-0.500;
                q.y=0.500;
                q.z=-0.500;
                q.w=0.500;

                RCLCPP_INFO_STREAM(node_->get_logger(), "[updateECCTransform] Updated transform! " << stamp.sec << "." << stamp.nanosec);

                ecc_.header.stamp.nanosec += nano_second.nanosec;
                buffer_->setTransform(ecc_, "coordinate_frame_helper", false);
                ecc_.header.stamp.nanosec -= nano_second.nanosec;
                tf_br_->sendTransform(ecc_);

                return true;
            }

            bool updateOffsetTransform(builtin_interfaces::msg::Time stamp)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "[updateOffsetTransform]");

                if (new_cfd_)
                {
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "[updateOffsetTransform] Have new CoordinateFrameDefinition!");
                    
                    geometry_msgs::msg::PoseStamped des_origin_pose;
                    try
                    {
                        rclcpp::Time rosTime = rclcpp::Time(stamp);
                        auto trans = buffer_->lookupTransform(getECSFrameId(), fixed_frame_id_, rosTime); // cfd_.pose,
                        tf2::doTransform(cfd_.pose, des_origin_pose, trans);
                    }
                    catch (tf2::TransformException &ex) 
                    {
                        RCLCPP_WARN_STREAM(node_->get_logger(), "Problem finding transform:\n" <<ex.what());
                        return false;
                    }
                    
                    if(0) //Disable this until have use for it
                    {
                        auto& t = offset_transform_.transform.translation;
                        const auto& pt = des_origin_pose.pose.position;
                        
                        t.x = pt.x;
                        t.y = pt.y;
                        t.z = pt.z;
                        
                        // For now, this likely represents an error
                        if (t.x != 0 || t.y != 0 || t.z != 0)
                        {
                            RCLCPP_WARN_STREAM(node_->get_logger(), "[update_offset_transform] The specified offset has a non-zero translational component, are you sure you want to do this?");
                        }
                    }
                    
                    //TODO: Ensure that only 'yaw' is captured
                    
                    offset_transform_.transform.rotation = des_origin_pose.pose.orientation;
                    offset_transform_.header.frame_id =  getECSFrameId();
                    offset_transform_.header.stamp = des_origin_pose.header.stamp;
                    offset_transform_.child_frame_id = getECFrameId();
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully processed new offset definition!");

                    new_cfd_ = false;
                }
                else
                {                    
                    if (stamp == offset_transform_.header.stamp && stamp != builtin_interfaces::msg::Time())
                    {
                        RCLCPP_WARN_STREAM(node_->get_logger(), "[updateOffsetTransform] Not publishing redundant transform! " << stamp.sec << "." << stamp.nanosec);
                        return true;
                    }
                    offset_transform_.header.stamp = stamp;
                }

                RCLCPP_DEBUG_STREAM(node_->get_logger(), "[updateOffsetTransform] Updated transform! " << stamp.sec << "." << stamp.nanosec);
                offset_transform_.header.stamp.nanosec += nano_second.nanosec;
                buffer_->setTransform(offset_transform_, "coordinate_frame_helper", false);
                offset_transform_.header.stamp.nanosec -= nano_second.nanosec;
                tf_br_->sendTransform(offset_transform_);

                return true;
            }
          
        };
    } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_COORDINATE_FRAME_HELPER_H

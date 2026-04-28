#pragma once

// #include <ros/node_handle.h>
#include <rclcpp/rclcpp.hpp>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

#include <mmp_quadruped/planning/ModeFamilyNode.h>
#include <mmp_quadruped/planning/TorsoStateNode.h>
#include <mmp_quadruped/planning/Superquadric.h>
#include <mmp_quadruped/visualization/Go2Visualizer.h>

// #include "ocs2_quadruped/constraint/EndEffectorLinearConstraint.h"

#include <visualization_msgs/msg/marker_array.hpp>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <geometry_msgs/msg/point.h>

namespace mmp {

class MultiModalVisualizer 
{
    public:
        MultiModalVisualizer(const rclcpp::Node::SharedPtr &node, 
                                std::shared_ptr<Go2Visualizer> & go2Visualizer);

        void launchVisualizerNode(const rclcpp::Node::SharedPtr &node);
      
        void visualizeSuperquadrics(const std::vector<Superquadric * > & superquadrics,
                                    const base_coordinate_t & torso_pose);

        void publishLead(std::vector<ModeFamilyNode::Transition *> & lead);
        void publishTorsoLocalWaypoint(const base_coordinate_t & torsoLocalWaypointPose,
                                        const double & localGoalRegionTolerance);       
        void publishTorsoInitPos(const base_coordinate_t & torsoInitPos,
                                    const double & initPosRegionTolerance);                                         
        void publishTorsoGlobalGoal(const base_coordinate_t & torsoGlobalGoalPosition,
                                    const double & globalGoalRegionTolerance);
        void publishTorsoPath(std::vector<TorsoStateNode::TorsoTransition *> torsoPath);
        void visualizeReferenceTrajectory(const vector_array_t & state_trajectory, 
                                            const vector_array_t & input_trajectory,
                                            const scalar_array_t & time_trajectory,
                                            const double & real_time_factor,
                                            const double & num_path_visualizations);       
        void visualizePath(const vector_array_t & state_trajectory, 
                            const vector_array_t & input_trajectory,
                            const scalar_array_t & time_trajectory,                  
                            double real_time_factor = 1.0, 
                            double num_path_visualizations = 1.0);
        // void visualizePathWithPolygons(MmpInterface & interface,
        //                                 const vector_array_t & state_trajectory, 
        //                                 const vector_array_t & input_trajectory,
        //                                 const scalar_array_t & time_trajectory,
        //                                 const double & timeHorizon,         
        //                                 const std::vector<ModeFamilyNode::Transition *> & lead,                                                            
        //                                 double real_time_factor = 1.0,
        //                                 double num_path_visualizations = 1.0);                       
        void visualizeFailedStep(const vector_array_t & failed_step_state_trajectory,
                                    const vector_array_t & failed_step_input_trajectory,
                                    const scalar_array_t & failed_step_time_trajectory);
        void visualizePathNTimes(const vector_array_t & state_trajectory, 
                                    const vector_array_t & input_trajectory,
                                    const scalar_array_t & time_trajectory,                                                      
                                    double real_time_factor);
        // void visualizePathWithPolygonsNTimes(MmpInterface & interface,
        //                                         const vector_array_t & state_trajectory, 
        //                                         const vector_array_t & input_trajectory,
        //                                         const scalar_array_t & time_trajectory,
        //                                         const double & timeHorizon,         
        //                                         const std::vector<ModeFamilyNode::Transition *> & lead,                                                            
        //                                         double real_time_factor);                                  
    private:

        // void visualizeShrinkingPolygons(const std::vector<ModeFamilyNode::Transition *> & lead,
        //                                 const int & step_counter,
        //                                 const double & currTime);
        // quadruped::EndEffectorLinearConstraint::Config populateConfig(const double & time,
        //                                                                 const vector3_t & bot_left_pt_,
        //                                                                 const vector3_t & bot_right_pt_,
        //                                                                 const vector3_t & top_left_pt_,
        //                                                                 const vector3_t & top_right_pt_,
        //                                                                 const vector3_t & mid_pt_,
        //                                                                 const double & slack_factor);

        double timeStep_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr leadPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr torsoPathPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr superquadricPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shrinkingPolygonsPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr torsoLocalGoalPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr torsoGlobalGoalPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr torsoInitPosPublisher_;

        vector3_t inf_bot_left_pt_, inf_bot_right_pt_, inf_top_left_pt_, inf_top_right_pt_;        
        double FL_slack_factor = 0.0, FR_slack_factor = 0.0, BL_slack_factor = 0.0, BR_slack_factor = 0.0;

        feet_array_t<ocs2::Color> feetColorMap_ = {ocs2::Color::blue, ocs2::Color::orange, ocs2::Color::yellow,
                                                    ocs2::Color::purple};  // Colors for markers per feet
        std::shared_ptr<Go2Visualizer> go2Visualizer_;

        rclcpp::Node::SharedPtr node_; /**< The ROS node handle */
};

}  // namespace mmp

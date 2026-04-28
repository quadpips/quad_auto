// #include <mmp_quadruped/MmpInterface.h>
#include <mmp_quadruped/planning/ContactSequence.h>

#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>
#include <ocs2_switched_model_msgs/msg/scheduled_gait_sequence.h>
#include <ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h>

#include <ocs2_core/reference/TargetTrajectories.h>

// #include <ocs2_quadruped/environment/TerrainProfile.h>

// #include <ocs2_quadruped_ros/gait/ModeSequenceTemplateRos.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

// #include <ocs2_quadruped_ros/environment/TerrainProfileRos.h>

#include <chrono>


namespace mmp {

class ReferenceTrajectoryPublisher
{
    public:
        /**
        * @brief Construct a new Reference Trajectory Publisher object
        * 
        * @param nh ROS node handle
        * @param mpc_topic_prefix Prefix for the MPC ROS topics
        */
        ReferenceTrajectoryPublisher(const rclcpp::Node::SharedPtr& node, 
                                     const std::string & mpc_topic_prefix,
                                        const std::string & hyperparametersFile,
                                        const std::string & gait);

        /**
        * @brief Prepare the reference trajectory to publish to MPC for tracking
        *
        * @param state_trajectory State trajectory
        * @param input_trajectory Input trajectory
        * @param time_trajectory Time trajectory
        * @param mode_schedules Mode schedules
        * @param switching_times Switching times
        */
        void sendReferenceTrajectory(const ocs2::TargetTrajectories & inc_ref_trajectory,
                                        const switched_model::GaitSchedule::GaitSequence & gaitSequence);

        /**
        * @brief Check if tracking is done based on the final state of the reference trajectory
        *
        * @return true if tracking is done, false if not
        */
        bool isTrackingDoneStateBased();

        // /**
        // * @brief Stop reference trajectory tracking by clearing the current reference trajectory
        // *        and sending a long stance phase for the current state+input
        // *
        // * @param current_t Current time
        // * @param current_q Current state
        // * @param current_u Current input
        // */ 
        // void stopTracking(const double & current_t,
        //                     const Eigen::VectorXd & current_q, 
        //                     const Eigen::VectorXd & current_u);

        std::pair<ocs2::TargetTrajectories, switched_model::GaitSchedule::GaitSequence> prepareReferenceTrajectory(const vector_array_t & state_trajectory, 
                                    const vector_array_t & input_trajectory, 
                                    const scalar_array_t & time_trajectory, 
                                    const std::vector<std::string> & mode_schedules,
                                    const std::vector<std::string> & switching_times);

        bool refineTO() const { return params_.refine_to; }
        bool fullRefTraj() const { return params_.full_ref_traj; }

    private:

        struct ReferenceTrackingParams
        {
            // 5.0 sec : works well
            // 2.5 sec : works well
            // 0.0 sec : fails
            // 0.5 sec : can be too short
            scalar_t T_ahead = 0.0; // decided by the delay on the gaitReceiver

            bool include_inputs = false; // whether to include inputs (contact forces and joint velocities) in reference trajectory
            bool refine_to = false; // whether to refine long horizon TO reference trajectory (not really working currently)
            bool full_ref_traj = true; // whether to send full reference trajectory (stance + swing phases)
            double near_goal_dist_threshold = 0.0; // distance to final goal position in reference trajectory to consider position constraint met
        } params_;


        ocs2::SystemObservation latest_observation_; /** Latest observation */
        
        rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr target_trajectories_publisher_; /** Target trajectories publisher */
        rclcpp::Publisher<ocs2_switched_model_msgs::msg::ScheduledGaitSequence>::SharedPtr gaitSequencePublisher_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_sub_; /** Observation subscriber */

        rclcpp::Node::SharedPtr node_; /** ROS node handle */

        std::string taskFile_; /** Task file */

        std::string gait_; /** Gait type */

        vector_t finalState_; /** Final state of reference trajectory, used in isTrackingDoneStateBased() */

        vector_t stableContactForces_; /**< Stable contact force, nominal contact forces during stance phase, used in isTrackingDoneStateBased() */ 
        vector_t stableFootVelocity_; /**< Stable foot velocity, nominal foot velocities during stance phase, used in isTrackingDoneStateBased() */

};

}  // namespace mmp

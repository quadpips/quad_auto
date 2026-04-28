#pragma once

#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mmp_quadruped/environment/EnvironmentProcessor.h>

#include <mmp_quadruped/planning/Graph.h>
#include <mmp_quadruped/planning/ConstraintManager.h>
#include <mmp_quadruped/planning/GraphSearcher.h>
#include <mmp_quadruped/planning/GraphTraverser.h>

#include <mmp_quadruped/planning/ReferenceTrajectoryPublisher.h>
#include <mmp_quadruped/visualization/MultiModalVisualizer.h>
#include <mmp_quadruped/planning/Superquadric.h>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <boost/circular_buffer.hpp>

// Include transforms
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace mmp {

class MultiModalPlanner
{
    public:
        /**
        * @brief Construct a new MultiModalPlanner object
        *
        * @param node The ROS node handle
        * @param multiModalVisualizer The multi-modal planning visualizer
        * @param interface The legged robot interface
        * @param hyperparametersFile The hyperparameters file
        * @param envFile The environment file
        * @param gaitCommandFile The gait command file
        * @param gait The gait
        * @param timeHorizon The time horizon for the planner
        */
        MultiModalPlanner(const rclcpp::Node::SharedPtr& node, 
                            std::shared_ptr<MultiModalVisualizer> & multiModalVisualizer, 
                            std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface, 
                            const std::string & taskFile,
                            const std::string & sqpFile,    
                            const std::string & hyperparametersFile, 
                            const std::string & envFile,
                            const std::string & gaitCommandFile,
                            const std::string & gait);

        ~MultiModalPlanner();

        /**
        * @brief run the planning loop
        *
        * @return true if the planner was successful
        */
        bool runPlanner(const std::shared_ptr<ocs2::SqpSolver> & solverPtr);

        /**
        * @brief Track the plan
        *
        */
        void trackPlan(const std::shared_ptr<ocs2::SqpSolver> & solverPtr, const bool & success);

        /**
        * @brief Prepare the reference trajectory for publishing
        * @return pair of target trajectories and gait sequence
        */
        void prepareReferenceTrajectory(const std::shared_ptr<ocs2::SqpSolver> & solverPtr);

        /**
        * @brief Publish the reference trajectory
        */
        void publishReferenceTrajectory();
        
        /**
        * @brief Monitor the reference trajectory to determine if we must replan
        */
        void monitorReferenceTrajectory();

        /**
        * @brief Check if the planner is ready to plan
        *
        * @return true if the planner is ready to plan
        */
        bool readyToPlan();

        bool reachedGlobalGoal() { return metGlobalGoal_; }

        void clear();
        
        bool generatedPlan();

        void setStateAndInput();

        void checkGlobalGoalStatus();

        ocs2::SystemObservation getLatestObservation();

    private:

        void initializeGraph(const comkino_state_t & latest_x,
                                const comkino_input_t & latest_u,
                                const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                               const int & torsoLocalWaypointIdx);

        /**
        * @brief Run the offline planning phase
        *
        * @param timeHorizon The time horizon for the planner
        * @param torsoPath The guiding torso path
        * @return true if the planner was successful
        */
        bool offlinePlanning(const comkino_state_t & latest_x,
                                const comkino_input_t & latest_u,
                                const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                                const std::shared_ptr<ocs2::SqpSolver> & solverPtr);

        void observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg);

        TorsoPathPlanner * tpp_ = NULL; /**< The torso path planner */
        EnvironmentProcessor * ep_ = NULL; /**< The environment processor */
        ConstraintManager * cm_ = NULL; /**< The constraint manager */
        GraphSearcher * gs_ = NULL; /**< The graph searcher */
        Graph * graph_ = NULL; /**< The graph */
        GraphTraverser * gt_ = NULL; /**< The graph traverser */
        ReferenceTrajectoryPublisher * refTrajPublisher_ = NULL; /**< The reference trajectory publisher */

        std::shared_ptr<switched_model::CustomQuadrupedInterface> interface_; /**< Go2 interface */

        std::vector<Superquadric * > superquadrics_; /**< superquadrics for each leg, used as reachability volumes */

        // std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
        // std::unique_ptr<tf2_ros::Buffer> tfBuffer_{nullptr};
        // std::shared_ptr<rclcpp::Clock> clock_{nullptr};
        // int failureCounter_ = 0; /**< Counter for the number of consecutive planning failures */

        bool metGlobalGoal_ = false; /**< Boolean for if global goal has been reached */

        std::vector<TorsoStateNode::TorsoTransition *> torsoPath;

        scalar_t timeHorizon_; /**< The time horizon for the TO */

        base_coordinate_t torsoGlobalGoalPose = base_coordinate_t::Zero(); /**< The torso global goal position in world frame */
        base_coordinate_t torsoLocalWaypointPose = base_coordinate_t::Zero(); /**< The torso local waypoint position in world frame */

        base_coordinate_t torsoInitPos = base_coordinate_t::Zero(); /**< The torso initial position in world frame */

        // Eigen::VectorXd latest_x; /**< The latest state */
        // Eigen::VectorXd latest_u; /**< The latest input */

        ocs2::TargetTrajectories latestReferenceTrajectory_; /**< The latest reference trajectory */
        switched_model::GaitSchedule::GaitSequence latestGaitSequence_; /**< The latest gait sequence */

        short search_algorithm = 1; /**< 0: Dijkstra's, 1: A* */
        short refine_to = 1; /**< 0: no refinement, 1: refinement enabled */
        short full_ref_traj = 1; /**< 0: sparse reference trajectory, 1: full reference trajectory */

        std::string taskFile_ = ""; /**< The task file */
        std::string sqpFile_ = ""; /**< The SQP file */
        std::string hyperparametersFile_ = ""; /**< The hyperparameters file */
        std::string envFile_ = ""; /**< The environment file */
        std::string referenceFile_ = ""; /**< The reference file */
        std::string gaitCommandFile_  = ""; /**< The gait command file */
        std::string gait_ = ""; /**< The gait */
        rclcpp::Node::SharedPtr node_; /**< The ROS node handle */

        mutable std::mutex observationMutex_;
        ocs2_msgs::msg::MpcObservation::ConstSharedPtr latestObservationPtr_ = nullptr; /**< latest observation from OCS2 as a ROS message */
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_; /**< ROS subscriber for the observation */
        std::shared_ptr<MultiModalVisualizer> multiModalVisualizer_; /**< The multi-modal planning visualizer */

};

}  // namespace mmp
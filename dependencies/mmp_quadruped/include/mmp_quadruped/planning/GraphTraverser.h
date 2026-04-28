#pragma once

#include <mmp_quadruped/planning/ConstraintManager.h>

#include <mmp_quadruped/planning/Graph.h>

#include <mmp_quadruped/planning/ExperienceStep.h>

#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_sqp/SqpSolver.h>
#include "ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h"
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"
#include "ocs2_switched_model_interface/logic/GaitSwitching.h"
#include <ocs2_core/reference/ModeSchedule.h>

namespace mmp {

class GraphTraverser
{
    public:

        /**
        * @brief Construct a new Graph Traverser object
        * 
        * @param cm pointer to the constraint manager object
        * @param interface pointer to the legged robot interface object
        */
        GraphTraverser(const rclcpp::Node::SharedPtr& node,
                        ConstraintManager * cm, 
                        std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface,
                        const std::string & sqpFile,
                        const std::string & taskFile,
                        const std::string & gaitCommandFile,
                        const std::string & gait);                        

        /**
        * @brief Setter for Graph object
        *
        * @param graph pointer to the graph object
        */
        void setGraph(Graph * graph) { graph_ = graph; }

        /**
        * @brief Function for rolling out lead and running trajectory optimization along with accumulating experience
        *
        * @param timeHorizon time horizon for the trajectory optimization
        * @param modeSearch boolean flag for mode search
        * @param updateWeights boolean flag for updating weights
        * @return true if successful, false otherwise
        */
        bool leadRollout(const std::shared_ptr<ocs2::SqpSolver> & solverPtr);

        // /**
        // * @brief Transform trajectories from base frame to world frame
        // *
        // * @param baseFrameToWorldFrameTransform transform from base frame to world frame
        // */
        // void transformTrajectories(const geometry_msgs::msg::TransformStamped & baseFrameToWorldFrameTransform);

        /**
        * @brief Clear trajectories from prior SQP run
        */
        void clearTrajectories();

        /**
        * @brief Clear experience buffers from prior SQP run
        */  
        void clearExperienceBuffers();

        void refineTO(const std::shared_ptr<ocs2::SqpSolver> & solverPtr,
                        ocs2::TargetTrajectories & targetTrajectories,
                        const GaitSchedule::GaitSequence & gaitSequence);        

        /**
        * @brief Getter for experience walks
        *
        * @return vector of experience steps from prior lead rollout
        */
        std::vector<ExperienceStep *> getExperienceWalk() { return experienceWalk; }

        /**
        * @brief Getter for state trajectory in base frame
        *
        * @return vector array of state trajectory in base frame
        */
        vector_array_t getStateTrajectoryBaseFrame() { return state_trajectory_base_frame; }
        
        /**
        * @brief Getter for input trajectory in base frame
        *
        * @return vector array of input trajectory in base frame
        */
        vector_array_t getInputTrajectoryBaseFrame() { return input_trajectory_base_frame; }
        
        /**
        * @brief Getter for time trajectory in base frame
        *
        * @return scalar array of time trajectory in base frame
        */
        scalar_array_t getTimeTrajectoryBaseFrame() { return time_trajectory_base_frame; }

        /**
        * @brief Getter for state trajectory in world frame
        *
        * @return vector array of state trajectory in world frame
        */
        vector_array_t getStateTrajectoryWorldFrame() { return state_trajectory_world_frame; }
        
        /**
        * @brief Getter for input trajectory in world frame
        *
        * @return vector array of input trajectory in world frame
        */
        vector_array_t getInputTrajectoryWorldFrame() { return input_trajectory_world_frame; }
        
        /**
        * @brief Getter for time trajectory in world frame
        *
        * @return scalar array of time trajectory in world frame
        */
        scalar_array_t getTimeTrajectoryWorldFrame() { return time_trajectory_world_frame; }

        /**
        * @brief Getter for switching times
        *
        * @return vector of switching times
        */
        std::vector<std::string> getSwitchingTimes() { return switching_times; }
        
        /**
        * @brief Getter for mode schedules
        *
        * @return vector of mode schedules
        */
        std::vector<std::string> getModeSchedules() { return mode_schedules; }

        /**
        * @brief Getter for failed step state trajectory in base frame
        *
        * @return vector array of failed step state trajectory in base frame
        */
        vector_array_t getFailedStepStateTrajectoryBaseFrame() { return failed_step_state_trajectory_base_frame; }
        
        /**
        * @brief Getter for failed step input trajectory in base frame
        *
        * @return vector array of failed step input trajectory in base frame
        */
        vector_array_t getFailedStepInputTrajectoryBaseFrame() { return failed_step_input_trajectory_base_frame; }
        
        /**
        * @brief Getter for failed step time trajectory in base frame
        *
        * @return scalar array of failed step time trajectory in base frame
        */
        scalar_array_t getFailedStepTimeTrajectoryBaseFrame() { return failed_step_time_trajectory_base_frame; }

        /**
        * @brief Getter for failed step state trajectory in world frame
        *
        * @return vector array of failed step state trajectory in world frame
        */
        vector_array_t getFailedStepStateTrajectoryWorldFrame() { return failed_step_state_trajectory_world_frame; }
        
        /**
        * @brief Getter for failed step input trajectory in world frame
        *
        * @return vector array of failed step input trajectory in world frame
        */
        vector_array_t getFailedStepInputTrajectoryWorldFrame() { return failed_step_input_trajectory_world_frame; }
        
        /**
        * @brief Getter for failed step time trajectory in world frame
        *
        * @return scalar array of failed step time trajectory in world frame
        */
        scalar_array_t getFailedStepTimeTrajectoryWorldFrame() { return failed_step_time_trajectory_world_frame; }

        /**
        * @brief Getter for terrain normals trajectory in base frame
        *
        * @return vector array of terrain normals trajectory in base frame
        */
        vector_array_t getTerrainNormalsTrajectoryBaseFrame() { return terrain_normals_trajectory_base_frame; }

        /**
        * @brief Getter for terrain normals trajectory in world frame
        *
        * @return vector array of terrain normals trajectory in world frame
        */
        vector_array_t getTerrainNormalsTrajectoryWorldFrame() { return terrain_normals_trajectory_world_frame; }

        ocs2::TargetTrajectories getRefinedTargetTrajectories() { return refinedTargetTrajectories_; }
        GaitSchedule::GaitSequence getRefinedGaitSequence() { return refinedGaitSequence_; }

        ocs2::TargetTrajectories getSparseTargetTrajectories() { return sparseTargetTrajectories_; }
        GaitSchedule::GaitSequence getSparseGaitSequence() { return sparseGaitSequence_; }

        std::vector<int> costBuckets; /**< cost buckets, used for recording cost distribution over all attempted transitions */
        std::vector<int> costCounts; /**< cost counts, used for recording cost distribution over all attempted transitions */

        std::vector<int> iterBuckets; /**< iteration buckets, used for recording iteration distribution over all attempted transitions */
        std::vector<int> iterCounts; /**< iteration counts, used for recording iteration distribution over all attempted transitions */
    private:

        /**
        * @brief Set up MMP interface before running solver
        */
        void setUpInterface();                            
        // const scalar_t & timeHorizon, 
        // const scalar_t & initialSwingTime, 
        // const scalar_t & finalSwingTime,
        // std::vector<std::vector<vector3_t>> & from_region_points,
        // std::vector<std::vector<vector3_t>> & to_region_points

        /**
        * @brief Record SQP stats (cost and iterations per step)
        * @param es : experience step where SQP was run
        */
        void recordSQPStats(ExperienceStep * es);

        /**
        * @brief Append SQP solution to reference trajectory
        * @param primalSolution : SQP solution
        * @param initialSwingTime : initial swing time
        * @param finalSwingTime : final swing time
        */
        void appendToReferenceTrajectory(const ocs2::PrimalSolution & primalSolution,
                                            const scalar_t & initialSwingTime,
                                            const scalar_t & finalSwingTime);

        void appendToSparseReferenceTrajectory(const ocs2::TargetTrajectories & targetTrajectories);

        // /**
        // * @brief Append from and to region normals to terrain profile
        // * @param primalSolution : SQP solution
        // * @param from_region_points : from region points
        // * @param to_region_points : to region points
        // */
        // void appendToTerrainProfile(const ocs2::PrimalSolution & primalSolution,
        //                             const std::vector<std::vector<vector3_t>> & from_region_points,
        //                             const std::vector<std::vector<vector3_t>> & to_region_points);

        /**
        * @brief Run SQP solver
        *
        * @param path vector of experience steps
        * @return true if successful, false otherwise
        */
        bool runSQP(std::vector<ExperienceStep *> & path,
                    const std::shared_ptr<ocs2::SqpSolver> & solverPtr); 

        /**
        * @brief Add failed step to failed step trajectories
        *
        * @param sqpSolution SQP solution
        */
        void addFailedStep(const ocs2::PrimalSolution & sqpSolution);

        rclcpp::Node::SharedPtr node_; /**< pointer to the ROS2 node */
        Graph * graph_ = NULL; /**< pointer to the graph object */
        ConstraintManager * constraintManager_ = NULL; /**< pointer to the constraint manager object */
        std::shared_ptr<switched_model::CustomQuadrupedInterface> interface_; /**< pointer to the legged robot interface object */
        
        std::string gaitCommandFile_ = "";
        std::string gait_ = "";

        scalar_t initialSwingTime_ = 0.0;
        scalar_t finalSwingTime_ = 0.0;

        // ocs2::SqpSolver solver_; /**< SQP solver */

        scalar_t timeHorizon_ = 0.0; /**< time horizon for the planner */

        ocs2::sqp::Settings sqpSettings; /**< SQP settings */

        vector_array_t state_trajectory_base_frame; /**< state trajectory in base frame */
        vector_array_t input_trajectory_base_frame; /**< input trajectory in base frame */
        scalar_array_t time_trajectory_base_frame; /**< time trajectory in base frame */

        vector_array_t state_trajectory_world_frame; /**< state trajectory in world frame */
        vector_array_t input_trajectory_world_frame; /**< input trajectory in world frame */
        scalar_array_t time_trajectory_world_frame; /**< time trajectory in world frame */

        vector_array_t failed_step_state_trajectory_base_frame; /**< failed step state trajectory in base frame */
        vector_array_t failed_step_input_trajectory_base_frame; /**< failed step input trajectory in base frame */
        scalar_array_t failed_step_time_trajectory_base_frame; /**< failed step time trajectory in base frame */

        vector_array_t failed_step_state_trajectory_world_frame; /**< failed step state trajectory in world frame */
        vector_array_t failed_step_input_trajectory_world_frame; /**< failed step input trajectory in world frame */
        scalar_array_t failed_step_time_trajectory_world_frame; /**< failed step time trajectory in world frame */

        vector_t failed_transition_state; /**< failed transition state */

        vector_array_t terrain_normals_trajectory_base_frame; /**< terrain normals trajectory in base frame */
        vector_array_t terrain_normals_trajectory_world_frame; /**< terrain normals trajectory in world frame */

        std::vector<std::string> switching_times; /**< aggregate switching times across entire reference trajectory */
        std::vector<std::string> mode_schedules; /**< aggregate mode schedules across entire reference trajectory */
        std::vector<ExperienceStep *> experienceWalk; /**< vector of experience steps from prior lead rollout */

        ocs2::TargetTrajectories sparseTargetTrajectories_; /**< refined target trajectories after TO refinement */
        GaitSchedule::GaitSequence sparseGaitSequence_; /**< refined gait sequence after TO refinement */
        int sparseCounter_ = 0;

        ocs2::TargetTrajectories refinedTargetTrajectories_; /**< refined target trajectories after TO refinement */
        GaitSchedule::GaitSequence refinedGaitSequence_; /**< refined gait sequence after TO refinement */
};

}  // namespace mmp


// #endif
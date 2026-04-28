#pragma once

// #include <mmp_quadruped/environment/SteppableRegion.h>

// #include <convex_plane_decomposition/PlanarRegion.h>

#include <mmp_quadruped/planning/ContactSequence.h>
#include <mmp_quadruped/planning/ModeFamilyNode.h>
#include <mmp_quadruped/planning/ModeFamilyNodeID.h>
#include <mmp_quadruped/planning/Superquadric.h>
#include <mmp_quadruped/planning/TorsoPathPlanner.h>
#include <mmp_quadruped/planning/utils.h>

#include <map>
#include <chrono>


namespace mmp {

/****************************Graph****************************/
class Graph
{
    public:
        /**
        * @brief Construct a new Graph object
        * 
        * @param search_algorithm flag for if we are using A* (1) or Dijkstra (0)
        * @param gaitCommandFile file containing gait commands
        * @param gait gait name
        * @param regions local steppable regions to use in graph
        * @param hyperparametersFile file containing planning hyperparameter values
        * @param torsoPath guiding torso path to use in graph construction
        */
        Graph(const rclcpp::Node::SharedPtr& node,
                const comkino_state_t & latest_x,
                const short & search_algorithm, 
                const std::string & gaitCommandFile, 
                const std::string & gait,
                const std::vector<switched_model::ConvexTerrain> & regions,
                const std::string & hyperparametersFile,
                const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                const std::vector<Superquadric * > & superquadrics);

        /**
        * @brief Destroy the Graph object
        * 
        */
        ~Graph();

        /**
        * @brief Add a start node to the graph
        *
        * @param startStanceID ID of start stance node
        * @param torsoPath guiding torso path to use in graph construction
        */
        void addStartNode(const ModeFamilyNodeID & startStanceID, 
                            const base_coordinate_t & refTorsoPose,
                            const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath);

        /**
        * @brief Add a goal node to the graph
        *
        * @param torsoLocalWaypointPosition The torso local waypoint position in world frame
        */
        void addGoalNode(ModeFamilyNode * currentNode);

        void addNodesV2(ModeFamilyNode * candidateNode,
                        const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath);  

        /**
        * @brief Check if the current graph transition is a discrete mode switch
        *
        * @return true if discrete mode switch, false otherwise
        */
        bool checkDiscreteModeSwitch();

        /**
        * @brief Check if the robot has reached the goal
        *
        * @return true if robot has reached goal, false otherwise
        */
        bool reachedGoal() { return (*currentNode->modeFamily == *goalNode->modeFamily); }

        // void planStep();

        /**
        * @brief Update the current graph transition prior to preparing for trajectory optimization
        * 
        */
        void updateCurrentTransition();

        /**
        * @brief Advance the lead counter forward one after a successful step
        * 
        */
        void advanceCounter();

        /**
        * @brief Reset the graph after a search is completed
        * 
        */
        void resetGraph();

        /**
        * @brief Getter for lead counter
        *
        * @return int lead counter
        */
        int getLeadCounter() { return lead_counter; }

        /**
        * @brief Getter for start node
        *
        * @return ModeFamilyNode* start node
        */
        ModeFamilyNode * getStartNode() { return startNode; }
        
        /**
        * @brief Getter for goal node
        *
        * @return ModeFamilyNode* goal node
        */
        ModeFamilyNode * getGoalNode() { return goalNode; }
        
        /**
        * @brief Getter for current node
        *
        * @return ModeFamilyNode* current node
        */
        ModeFamilyNode * getCurrentNode() { return currentNode; }

        void addTransition(ModeFamilyNode * nodeI, ModeFamilyNode * nodeJ,
                            const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath)
        {
            nodeI->addTransitionTo(nodeJ, torsoPath, w_exp, w_euc, w_stance, w_step, w_torso);
        }

        void addTransition(ModeFamilyNode * nodeI, ModeFamilyNode * nodeJ)
        {
            nodeI->addTransitionToGoal(nodeJ);
        }

        /**
        * @brief Getter for current transition
        *
        * @return ModeFamilyNode::Transition* current transition
        */
        ModeFamilyNode::Transition * getCurrentTransition() { return currentTransition; }
        
        /**
        * @brief Getter for previous transition
        *
        * @return ModeFamilyNode::Transition* previous transition
        */
        ModeFamilyNode::Transition * getPreviousTransition() { return previousTransition; }

        /**
        * @brief Getter for start mode family
        *
        * @return ModeFamily* start mode family
        */
        ModeFamily * getStartModeFamily() { return getStartNode()->modeFamily; }
        
        /**
        * @brief Getter for goal mode family
        *
        * @return ModeFamily* goal mode family
        */
        ModeFamily * getGoalModeFamily() { return getGoalNode()->modeFamily; }
        
        /**
        * @brief Getter for current mode family
        *
        * @return ModeFamily* current mode family
        */
        ModeFamily * getCurrentModeFamily() { return getCurrentNode()->modeFamily; }

        void printStats()
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "nodeMap.size(): " << nodeMap.size());
            RCLCPP_INFO_STREAM(node_->get_logger(), "edge_count: " << edge_count);
        }

        bool closeEnoughToLocalGoal(const vector3_t & torsoPosition) const
        {
            vector3_t localGoalPosition = extractTorsoPosition(localGoalPose_);
            // only doing x/y
            return (torsoPosition - localGoalPosition).norm() < localGoalRegionTolerance_;
        }

        void setLocalGoalRegion(const base_coordinate_t & torsoLocalWaypointPose);
        base_coordinate_t getLocalGoalRegion() { return localGoalPose_; }

        double getLocalGoalRegionTolerance() { return localGoalRegionTolerance_; }
        double getGlobalGoalRegionTolerance() { return globalGoalRegionTolerance_; }

        std::vector<Superquadric * > getSuperquadrics() { return superquadrics_; }

        std::vector<ModeFamilyNode::Transition *> lead; /**< lead of transitions obtained from graph search*/
        ContactSequence * contactSequence = NULL; /**< contact sequence object*/

        void printGraphInfo()
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "[Graph Info]");
            // RCLCPP_INFO_STREAM(node_->get_logger(), "  lead size: " << lead.size());
            RCLCPP_INFO_STREAM(node_->get_logger(), "  nodeMap size: " << nodeMap.size());
            RCLCPP_INFO_STREAM(node_->get_logger(), "  edge_count: " << edge_count);

            RCLCPP_INFO_STREAM(node_->get_logger(), "  FLReachableCount: " << FLReachableCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  FRReachableCount: " << FRReachableCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  BLReachableCount: " << BLReachableCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  BRReachableCount: " << BRReachableCount);            
            
            // RCLCPP_INFO_STREAM(node_->get_logger(), "  reachableCount: " << reachableCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  stableStanceCount: " << stableStanceCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  forwardFacingCount: " << forwardFacingCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  shortEnoughSwingCount: " << shortEnoughSwingCount);
            RCLCPP_INFO_STREAM(node_->get_logger(), "  perStepForwardProgressCount: " << perStepForwardProgressCount);

        }

        int64_t getGraphConstructionTime() { return graph_construction_time_; }

        bool isStableStance(const std::vector<vector3_t> & nominalPositions,
                            const base_coordinate_t & torso_pose_world_frame);

    private:

        bool isTransitionToPrevious(ModeFamilyNode * currentNode, ModeFamilyNode * nextNode);

        bool makingPerStepForwardProgress(const ModeFamilyNode * candidateNode, const std::vector<vector3_t> & nominalPositions);

        bool makingForwardProgress(const base_coordinate_t & nominalTorsoPose, const base_coordinate_t & candidateTorsoPose);

        bool isTransitionSteppingOnSameRegions(const ModeFamilyNode * candidateNode, const ModeFamilyNode * nextNode);

        bool isSwingShortEnough(const ModeFamilyNode * candidateNode, const std::vector<vector3_t> & nominalPositions);

        bool isForwardFacing(const base_coordinate_t & torsoPose);

        bool isReachable(const base_coordinate_t & torsoPose, 
                            const std::vector<switched_model::ConvexTerrain> & footRegions,
                            const bool & unitTestPrint);

        /**
        * @brief Create a node from coordinates
        *
        * @param nodeID node ID to create node from
        * @return ModeFamilyNode* created node
        */
        ModeFamilyNode * createNodeFromID(const ModeFamilyNodeID & nodeID,
                                            const base_coordinate_t & refTorsoPose);

        // /**
        // * @brief Create a goal node from a waypoint
        // *
        // * @param torsoLocalWaypointPosition The torso local waypoint position in world frame
        // * @return ModeFamilyNode* created goal node
        // */
        // ModeFamilyNode * createGoalNodeFromWaypoint(const vector3_t & torsoLocalWaypointPosition);

        /**
        * @brief Check if two regions are far enough away to disregard for graph construction
        *
        * @param regionAID ID of first region
        * @param regionBID ID of second region
        * @return true if regions are close enough, false otherwise
        */
        bool localRegionDistanceCheck(const int & regionAID, const int & regionBID);

        /**
        * @brief Get torso pose from regions
        *
        * @param regions regions to use for torso pose calculation
        * @return Eigen::VectorXd torso pose
        */
        base_coordinate_t getTorsoPose(const std::vector<vector3_t> & regions,
                                        const bool & unitTestPrint = false);

        // double calculateTorsoPathDeviation(const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
        //                                         const base_coordinate_t & nominalTorsoPose,
        //                                         const bool & unitTestPrint);

        rclcpp::Node::SharedPtr node_;
            
        std::map<ModeFamilyNodeID, ModeFamilyNode *> nodeMap; /**< map of mode family node IDs to mode family nodes */
        short search_algorithm = 0; /**< search algorithm flag (0 for Dijkstra, 1 for A*) */
        int lead_counter = 0; /**< lead counter for graph search */
        int edge_count = 0; /**< edge count for graph search */

        std::vector<switched_model::ConvexTerrain> regions_; /**< local regions used in graph construction */

        std::vector<Superquadric * > superquadrics_; /**< superquadrics for each leg, used as reachability volumes */

        base_coordinate_t localGoalPose_; /**< The goal position of the local goal mode family */
        double localGoalRegionTolerance_ = 0.0; // 0.15; /**< The tolerance for the local goal region */
        double globalGoalRegionTolerance_ = 0.0; // 0.30; /**< The tolerance for the global goal region */

        vector3_t torsoPosition_; /**< The position of the torso */
        vector3_t torsoOrientation_; /**< The orientation of the torso */

        int reachableCount = 0; /**< Count of reachable nodes */
        int stableStanceCount = 0; /**< Count of stable stance nodes */
        int forwardFacingCount = 0; /**< Count of forward facing nodes */
        int shortEnoughSwingCount = 0; /**< Count of nodes with short enough swing */
        int perStepForwardProgressCount = 0; /**< Count of nodes with per step forward progress */
        int FLReachableCount = 0; /**< Count of reachable nodes for FL leg */
        int FRReachableCount = 0; /**< Count of reachable nodes for FR leg */
        int BLReachableCount = 0; /**< Count of reachable nodes for BL leg */
        int BRReachableCount = 0; /**< Count of reachable nodes for BR leg */

        double w_exp = 0.0; /**< weight for experience term in transition */
        double w_euc = 0.0; /**< weight for euclidean distance term in transition */
        double w_stance = 0.0; /**< weight for stance deviation term in transition */
        double w_step = 0.0; /**< weight for step term in transition */
        double w_torso = 0.0; /**< weight for torso path deviation term in transition */

        ModeFamilyNode * startNode = NULL; /**< start node for graph search */
        ModeFamilyNode * goalNode = NULL; /**< goal node for graph search */
        ModeFamilyNode * currentNode = NULL; /**< current node for graph search */

        ModeFamilyNode::Transition * currentTransition = NULL; /**< current transition for graph search */
        ModeFamilyNode::Transition * previousTransition = NULL; /**< previous transition for graph search */

        int64_t graph_construction_time_ = 0; /**< tracking time for graph search */

        struct GraphConstructionParams
        {
            double max_leg_swing_distance;
            double ideal_stance_width;
            double ideal_stance_length;
        };

        GraphConstructionParams params_;
};

}  // namespace mmp

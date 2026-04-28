#pragma once

#include <map>
#include <cmath>
#include <limits>

#include <iostream>
#include <vector>

#include <mmp_quadruped/planning/TorsoHeap.h>
#include <mmp_quadruped/planning/TorsoStateNode.h>
#include <mmp_quadruped/planning/utils.h>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

namespace mmp {

class TorsoPathPlanner
{
    public:
        /**
        * @brief Construct a new Torso Path Planner object
        */
        TorsoPathPlanner(const rclcpp::Node::SharedPtr& node,
                            const std::string & hyperparametersFile);

        /**
        * @brief Destroy a Torso Path Planner object
        * 
        */
        ~TorsoPathPlanner();
     
        /**
        * @brief Build the torso state graph
        *
        * @param startCoM The starting CoM position to start the search from.
        * @param goalCoM The goal CoM position to end the search at.
        * @param minEnvPt The minimum point of the environment bounding box, defines the discretized search space
        * @param maxEnvPt The maximum point of the environment bounding box, defines the discretized search space
        * @param activeCollisionAvoidanceConstraint Whether or not to use the collision avoidance constraint
        */
        void buildStateGraph(const comkino_state_t & latest_x,
                                const base_coordinate_t & goalTorsoPose,
                                const vector3_t & minEnvPt,
                                const vector3_t & maxEnvPt);
                                // const std::vector<switched_model::ConvexTerrain> & localRegions);

        /**
        * @brief Run the torso path planner
        *
        * @return The torso path: a vector of torso transitions from start to goal
        */
        std::vector<TorsoStateNode::TorsoTransition *> runPlanner();

        /**
        * @brief Extract the torso local waypoint position based on the given configuration
        *
        * @param latest_q The latest configuration of the robot
        * @return The local torso waypoint position
        */
        int extractTorsoLocalWaypointPosition(const comkino_state_t & latest_x);

    private:
        
        void reset();
        void wipeNodes();

        /**
        * @brief Search over the torso state graph
        *
        * @return The trace node to search backwards from
        */
        TorsoStateNode * search();

        rclcpp::Node::SharedPtr node_; /**< ROS2 node pointer */

        double delta = 0.0; /**< The discretization step size for the torso state graph */

        bool addedStart = false; /**< Whether or not the start node was successfully added to the graph */
        bool addedGoal = false; /**< Whether or not the goal node was successfully added to the graph */

        TorsoStateNode * startNode_ = NULL; /**< The start node of the torso state graph */
        TorsoStateNode * goalNode_ = NULL; /**< The goal node of the torso state graph */

        std::vector<TorsoStateNode::TorsoTransition *> torsoPath; /**< Torso path */
        std::vector<TorsoStateNode *> nodeVector; /**< Vector of nodes in graph, used for memory maintenance */

        double local_search_radius_ = 0.0; /**< The radius for local graph search */

};


}  // namespace mmp

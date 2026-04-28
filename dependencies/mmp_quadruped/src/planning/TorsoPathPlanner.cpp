#include <mmp_quadruped/planning/TorsoPathPlanner.h>


namespace mmp {

/********* TorsoPathPlanner ********/
TorsoPathPlanner::TorsoPathPlanner(const rclcpp::Node::SharedPtr& node,
                                    const std::string & hyperparametersFile)
{
    node_ = node;
    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(hyperparametersFile, pt);
    const std::string prefix = "torso_path_planner.";
    ocs2::loadData::loadPtreeValue(pt, delta, prefix + "delta", verbose);
    ocs2::loadData::loadPtreeValue(pt, local_search_radius_, prefix + "local_search_radius", verbose);
}

TorsoPathPlanner::~TorsoPathPlanner()
{
    wipeNodes();
}

void TorsoPathPlanner::wipeNodes()
{
    for (auto & node : nodeVector)
        delete node;
    nodeVector.clear();
}

void TorsoPathPlanner::reset()
{
    wipeNodes();
    startNode_ = NULL;
    goalNode_ = NULL;
    addedStart = false;
    addedGoal = false;

    torsoPath.clear();
}

void TorsoPathPlanner::buildStateGraph(const comkino_state_t & latest_x,
                                        const base_coordinate_t & goalTorsoPose,
                                        const vector3_t & minEnvPt,
                                        const vector3_t & maxEnvPt)
                                        // const std::vector<switched_model::ConvexTerrain> & localRegions)
{
    // Make sure empty before start
    reset();
    
    vector3_t goalPosition = extractTorsoPosition(goalTorsoPose); // .head(3);
    vector3_t goalOrientation = extractTorsoOrientation(goalTorsoPose); // .tail(3);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[buildStateGraph()]");
    double min_start_dist = 1e6; // 1e30;
    double min_goal_dist = 1e6; // 1e30;

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    vector3_t startPosition = extractTorsoPosition(torso_pose);
    vector3_t startOrientation = extractTorsoOrientation(torso_pose);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      startPosition: " << startPosition.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      startOrientation: " << startOrientation.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      goalPosition: " << goalPosition.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      goalOrientation: " << goalOrientation.transpose());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      minEnvPt: " << minEnvPt.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      maxEnvPt: " << maxEnvPt.transpose());

    double minTheta = -M_PI;
    double maxTheta = M_PI;
    double deltaTheta = 0.125 * M_PI;

    for (double x = minEnvPt[0]; x <= maxEnvPt[0]; x += delta)
    {
        for (double y = minEnvPt[1]; y <= maxEnvPt[1]; y += delta)
        {
            for (double z = minEnvPt[2]; z <= maxEnvPt[2]; z += delta) // ad-hoc scaling up to increase search space
            {
    // for (int i = 0; i < localRegions.size(); i++)
    // {
    //     vector3_t regionPos = localRegions[i].plane.positionInWorld;
    //     double x = regionPos[0];
    //     double y = regionPos[1];
    //     double z = regionPos[2];
                for (double theta = minTheta; theta <= maxTheta; theta += deltaTheta)
                {
                    vector3_t torsoPosition(x, y, z + Z_TORSO_OFFSET);

                    // double theta = 0.0;        
                    vector3_t torsoOrientation(0.0, 0.0, theta);

                    // RCLCPP_INFO_STREAM(node_->get_logger(), "      torsoPosition: " << torsoPosition.transpose());
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "      torsoOrientation: " << torsoOrientation.transpose());

                    double goal_pos_dist = (torsoPosition - goalPosition).norm(); // std::sqrt( std::pow(torsoPosition[0] - goalPosition[0], 2) + std::pow(torsoPosition[1] - goalPosition[1], 2)); 
                    Eigen::Vector2d torsoHeading(cos(torsoOrientation[2]), sin(torsoOrientation[2]));
                    Eigen::Vector2d goalHeading(cos(goalOrientation[2]), sin(goalOrientation[2]));
                    double goal_ori_ang_diff = std::acos( torsoHeading.dot(goalHeading) / (torsoHeading.norm() * goalHeading.norm()) );
                    double goal_ori_dist = std::abs(goal_ori_ang_diff);
                    double goal_dist = goal_pos_dist + goal_ori_dist;
                    // bool isGoal = goal_pos_dist < min_goal_pos_dist;

                    TorsoStateNode * node = new TorsoStateNode(torsoPosition, torsoOrientation, goalPosition, goalOrientation); // , isGoal

                    double start_pos_dist = (torsoPosition - startPosition).norm(); // std::sqrt( std::pow(torsoPosition[0] - startPosition[0], 2) + std::pow(torsoPosition[1] - startPosition[1], 2)); 
                    Eigen::Vector2d startHeading(cos(startOrientation[2]), sin(startOrientation[2]));
                    double start_ori_ang_diff = std::acos( torsoHeading.dot(startHeading) / (torsoHeading.norm() * startHeading.norm()) );  
                    double start_ori_dist = std::abs(start_ori_ang_diff);
                    double start_dist = start_pos_dist + start_ori_dist;

                    // RCLCPP_INFO_STREAM(node_->get_logger(), "      i: " << i);
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "      start_dist: " << start_dist);
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "      goal_pos_dist: " << goal_pos_dist);

                    if (start_dist < min_start_dist)
                    {
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      setting p: " << torsoPosition.transpose() << ", o: " << torsoOrientation.transpose() << " as start node");
                        addedStart = true;
                        startNode_ = node;
                        min_start_dist = start_dist;
                    }

                    if (goal_dist < min_goal_dist)
                    {
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      setting p: " << torsoPosition.transpose() << ", o: " << torsoOrientation.transpose() << " as goal node");
                        addedGoal = true;
                        if (goalNode_)
                            goalNode_->setGoal(false);
                        goalNode_ = node;
                        goalNode_->setGoal(true);
                        min_goal_dist = goal_dist;
                    }

                    // std::string key = std::to_string(v_idx) + ", " + std::to_string(h_idx);
                    nodeVector.push_back(node);
                }
    //     }
    // }
            }
        }
    }

    if (nodeVector.size() == 0)
        throw std::runtime_error("Created zero nodes for torso graph");

    double max_posn_dist = std::sqrt( std::pow(delta, 2) + std::pow(delta, 2) + std::pow(delta, 2)); // allow diagonal transitions
    double max_ang_dist = 2 * deltaTheta;
    // double max_trans_dist = max_posn_dist + max_ang_dist + 0.01; // allow diagonal transitions

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  max_posn_dist: " << max_posn_dist);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  max_ang_dist: " << max_ang_dist);

    int edge_count = 0;
    for (TorsoStateNode * fromNode : nodeVector)
    {
        for (TorsoStateNode * toNode : nodeVector)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "    fromNode: " << fromNode->description());
            // RCLCPP_INFO_STREAM(node_->get_logger(), "    toNode: " << toNode->description());

            double posnDistance = fromNode->calculatePositionalDistance(toNode);
            double oriDistance = fromNode->calculateOrientationDistance(toNode);

            // RCLCPP_INFO_STREAM(node_->get_logger(), "      posnDistance: " << posnDistance);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "      oriDistance: " << oriDistance);

            // nodes are not the same, and within distance threshold
            if ((0 < posnDistance || 0 < oriDistance) &&
                posnDistance <= max_posn_dist &&
                oriDistance <= max_ang_dist)
            {
                fromNode->addTransition(toNode);
                edge_count++;
            }
        }
    }

    if (edge_count == 0)
        throw std::runtime_error("[TorsoPathPlanner] No edges created for torso graph");

    if (!addedStart)
        throw std::runtime_error("[TorsoPathPlanner] No start node");

    if (!addedGoal)
        throw std::runtime_error("[TorsoPathPlanner] No goal node");

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  Created " << nodeVector.size() << " nodes and " << edge_count << " edges for torso state graph");

    return;
}

std::vector<TorsoStateNode::TorsoTransition *> TorsoPathPlanner::runPlanner()
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "pre-search()");
    TorsoStateNode * traceNode = search();
    // RCLCPP_INFO_STREAM(node_->get_logger(), "post-search()");

    if (!traceNode->isGoal())
        throw std::runtime_error("[torsoSearch] Goal not found");

    while (traceNode)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "tracing node: " << traceNode->description() << 
                     ", node costToCome: " << traceNode->getCostToCome());
        // need to turn this into a transition
        // need a vector of transitions
        if (traceNode->previousTransition)
            torsoPath.insert(torsoPath.begin(), traceNode->previousTransition);
        else
            RCLCPP_INFO_STREAM(node_->get_logger(), "no previous node");

        traceNode = traceNode->previousNode;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "  torso path (size " << torsoPath.size() << "):");

    for (TorsoStateNode::TorsoTransition * tran : torsoPath)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "    from:" << tran->getNodeFrom()->description());
        RCLCPP_INFO_STREAM(node_->get_logger(), "    to:" << tran->getNodeTo()->description());
    }

    return torsoPath;
}

TorsoStateNode * TorsoPathPlanner::search()
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[TorsoPathPlanner::search()]");
    // pull existing code
    // need a another heap, TorsoHeap
    startNode_->setCostToCome(0.0);

    // visited: added to the frontier, but have not been popped. Their cost may still change during the graph search.

    startNode_->markAsVisited();
    
    TorsoHeap<TorsoStateNode> h(*startNode_); // just pass start node in as placeholder for 0-index    

    TorsoStateNode * currentNode = NULL;
    double candidateCostToCome;

    bool print = false;

    while (!h.empty())
    {
        /**
         * Pop the lowest cost node in the frontier and mark the node as explored
         */

        if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "    pre-pop");
        currentNode = h.pop();
        currentNode->markAsExplored();
        if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "    post-pop");

        /**
         * If the expanded node is the goal node, then search is done
         */
        if (currentNode->isGoal())
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "found goal, breaking");
            break;
            // return currentNode;
        }

        if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "  currentNode: " << currentNode->description() << ", cost: " << currentNode->getCostToCome());
        if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "  Number of transitions from currentNode: " << currentNode->trans.size());

        /**
         * Iterate through the popped node's transitions to see if we should either 
         *      a. add the destination node to the frontier (if it has not been visited)
         *      b. adjust the cost of the destination node (if it has been visited and is still in the frontier)
         */
        for (TorsoStateNode::TorsoTransition * tran : currentNode->trans)
        {
            if (!tran->getNodeTo()->isExplored()) // if destination node has not already been explored
            {
                candidateCostToCome = currentNode->getCostToCome() + tran->getWeight(); // "0.01" to avoid stepping on place

                /**
                 * If destination node of transition has NOT been visited yet
                 */
                if (!tran->getNodeTo()->isVisited())
                {
                    tran->getNodeTo()->setCostToCome(candidateCostToCome);
                    tran->getNodeTo()->previousNode = currentNode;
                    tran->getNodeTo()->previousTransition = tran;
                    tran->getNodeTo()->markAsVisited();

                    h.push(*tran->getNodeTo());
                } else
                {
                    /**
                     * If the node was added earlier with greater cost (didn't get expanded yet),
                     * then we need to update the cost (remove and add back to the queue) to 
                     * avoid sub-optimal and duplicate processings.
                     */
                    if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "NodeTo has been visited before with cost: " << currentNode->getCostToCome());
                    if (tran->getNodeTo()->getCostToCome() > candidateCostToCome)
                    {
                        // std::chrono::steady_clock::time_point updateCostToComeTimeBegin = std::chrono::steady_clock::now();
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "checking to remove modeFamilyFrontier");
                        
                        tran->getNodeTo()->previousNode = currentNode;
                        tran->getNodeTo()->previousTransition = tran;

                        int heapIdx = h.find(*tran->getNodeTo()); 
                        h.updateElem(heapIdx, candidateCostToCome);

                        // std::chrono::steady_clock::time_point updateCostToComeTimeEnd = std::chrono::steady_clock::now();
                        // updateCostToComeTime += 
                        //     std::chrono::duration_cast<std::chrono::microseconds>(updateCostToComeTimeEnd - updateCostToComeTimeBegin).count();        
                    }
                }
            }
            else
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "      has already been explored ");
            }
        }
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  updateCostToComeTime: " << updateCostToComeTime / 1.0e6 << " seconds");

    return currentNode;

}

int TorsoPathPlanner::extractTorsoLocalWaypointPosition(const comkino_state_t & latest_x) // 
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[TorsoPathPlanner::extractTorsoLocalWaypointPosition]");

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    vector3_t torso_position = extractTorsoPosition(torso_pose);

    RCLCPP_INFO_STREAM(node_->get_logger(), "  torso_position: " << torso_position.transpose());

    vector3_t torsoLocalWaypointPosition;
    int waypointIdx = -1;
    // iterate forward through torso path, maintaining waypoint as furthest position away within sensor radius
    for (TorsoStateNode::TorsoTransition * torsoTransition : torsoPath)
    {
        base_coordinate_t candidatePose = torsoTransition->getNodeTo()->getPoseWorldFrame();
        vector3_t candidatePosition = extractTorsoPosition(candidatePose);
        RCLCPP_INFO_STREAM(node_->get_logger(), "  candidate waypoint pose: " << candidatePose.transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "  candidate waypoint position: " << candidatePosition.transpose());

        double dist = (torso_position - candidatePosition).norm();

        RCLCPP_INFO_STREAM(node_->get_logger(), "  dist: " << dist);

        // should be in same frame here
        bool within_sensor_radius = (dist < local_search_radius_);
        
        if (within_sensor_radius) 
            waypointIdx++; 
        else
            break;
    }

    return waypointIdx;
}

}  // namespace mmp

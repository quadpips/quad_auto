#include <mmp_quadruped/planning/Graph.h>


namespace mmp {

/********************************Graph***********************************/
Graph::Graph(const rclcpp::Node::SharedPtr& node,
                const comkino_state_t & latest_x,
                const short & search_algorithm, 
                const std::string & gaitCommandFile, 
                const std::string & gait,
                const std::vector<switched_model::ConvexTerrain> & regions,
                const std::string & hyperparametersFile,
                const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                const std::vector<Superquadric * > & superquadrics)
{
    this->node_ = node;
    this->search_algorithm = search_algorithm;
    this->startNode = NULL;
    this->goalNode = NULL;
    this->currentNode = NULL;
    this->currentTransition = NULL;
    this->contactSequence = new ContactSequence(gaitCommandFile, gait);
    this->superquadrics_ = superquadrics;

    this->regions_ = regions;

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    this->torsoPosition_ = extractTorsoPosition(torso_pose);
    this->torsoOrientation_ = extractTorsoOrientation(torso_pose);

    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(hyperparametersFile, pt);
    const std::string prefix = "graph_weights.";
    ocs2::loadData::loadPtreeValue(pt, w_exp, prefix + "w_exp", verbose);
    ocs2::loadData::loadPtreeValue(pt, w_euc, prefix + "w_euc", verbose);
    ocs2::loadData::loadPtreeValue(pt, w_stance, prefix + "w_stance", verbose);
    ocs2::loadData::loadPtreeValue(pt, w_step, prefix + "w_step", verbose);
    ocs2::loadData::loadPtreeValue(pt, w_torso, prefix + "w_torso", verbose);

    const std::string graph_construction_prefix = "graph_construction.";
    ocs2::loadData::loadPtreeValue(pt, params_.max_leg_swing_distance, graph_construction_prefix + "max_leg_swing_distance", verbose);
    ocs2::loadData::loadPtreeValue(pt, params_.ideal_stance_width, graph_construction_prefix + "ideal_stance_width", verbose);
    ocs2::loadData::loadPtreeValue(pt, params_.ideal_stance_length, graph_construction_prefix + "ideal_stance_length", verbose);
    ocs2::loadData::loadPtreeValue(pt, localGoalRegionTolerance_, graph_construction_prefix + "local_goal_region_tolerance", verbose);
    ocs2::loadData::loadPtreeValue(pt, globalGoalRegionTolerance_, graph_construction_prefix + "global_goal_region_tolerance", verbose);

    RCLCPP_INFO_STREAM(node_->get_logger(), "max_leg_swing_distance: " << params_.max_leg_swing_distance);
    RCLCPP_INFO_STREAM(node_->get_logger(), "ideal_stance_width: " << params_.ideal_stance_width);
    RCLCPP_INFO_STREAM(node_->get_logger(), "ideal_stance_length: " << params_.ideal_stance_length);

    lead_counter = 0;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Graph instance created");
}

Graph::~Graph()
{
    // delete all graph nodes
    for (auto & node : nodeMap)
        delete node.second;

    delete contactSequence;

    nodeMap.clear();
    lead.clear();
}

bool Graph::localRegionDistanceCheck(const int & regionAID, const int & regionBID)
{
    vector3_t regionA = regions_[regionAID].plane.positionInWorld;
    vector3_t regionB = regions_[regionBID].plane.positionInWorld;

    double dist_thresh = 1.0;
    return (regionA - regionB).norm() > dist_thresh;
}

void Graph::setLocalGoalRegion(const base_coordinate_t & torsoLocalWaypointPose)
{
    localGoalPose_ = torsoLocalWaypointPose;
}

void Graph::addGoalNode(ModeFamilyNode * currentNode)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[Graph::addGoalNode]");

    ModeFamilyNodeID goalStanceID;

    for (int i = 0; i < 4; i++)
    {
        if (!currentNode->modeFamily->isLegInSwing(i))
        {
            goalStanceID.setRegionID(i, currentNode->modeFamily->getRegionID(i));
        } else
        {
            goalStanceID.setRegionID(i, currentNode->previousNode->modeFamily->getRegionID(i));            
        }
    }

    goalNode = createNodeFromID(goalStanceID, currentNode->getRefTorsoPose());

    nodeMap[goalStanceID] = goalNode;

    RCLCPP_INFO_STREAM(node_->get_logger(), "goalNode: " << goalNode->description());
}

void Graph::addNodesV2(ModeFamilyNode * currentNode,
                        const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[Graph::addNodesV2()]");

    std::chrono::steady_clock::time_point addNodeStartTime = std::chrono::steady_clock::now();

    ModeFamilyNodeID id;

    bool FL_swing = currentNode->modeFamily->isLegInSwing(FL);
    bool FR_swing = currentNode->modeFamily->isLegInSwing(FR);
    bool BL_swing = currentNode->modeFamily->isLegInSwing(BL);
    bool BR_swing = currentNode->modeFamily->isLegInSwing(BR);

    if (!FL_swing && !FR_swing && !BL_swing && !BR_swing)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "      all legs in stance, returning"); // manual fix for start node
        return;
    }

    std::vector<bool> swingFlags = {FL_swing, FR_swing, BL_swing, BR_swing};

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      FL_swing: " << FL_swing);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      FR_swing: " << FR_swing);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      BL_swing: " << BL_swing);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      BR_swing: " << BR_swing); 

    std::string nextMode = contactSequence->getNextMode(currentNode->modeFamily->getPhase());

    std::vector<bool> nextSwingFlags = {
        !(nextMode.find("LF") != std::string::npos),
        !(nextMode.find("RF") != std::string::npos),
        !(nextMode.find("LH") != std::string::npos),
        !(nextMode.find("RH") != std::string::npos)
    };

    bool next_FL_swing = nextSwingFlags[FL];
    bool next_FR_swing = nextSwingFlags[FR];
    bool next_BL_swing = nextSwingFlags[BL];
    bool next_BR_swing = nextSwingFlags[BR];

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      next_FL_swing: " << next_FL_swing);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      next_FR_swing: " << next_FR_swing);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      next_BL_swing: " << next_BL_swing);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      next_BR_swing: " << next_BR_swing);

    // bool passed_reachability = false;
    // bool passed_forward_facing = false;

    ModeFamilyNode * prevNode = currentNode->previousNode; // this should always exist
    std::vector<vector3_t> nominalPositions(4);

    for (int i = 0; i < 4; i++)
    {
        if (swingFlags[i])
        {
            nominalPositions[i] = prevNode->modeFamily->getNominalPositionWorldFrame(i);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "  nominalFootPosition for leg" << i << "(region ID: " << prevNode->modeFamily->getRegionID(i) << "): " << nominalPositions[i].transpose());
        } else
        {
            nominalPositions[i] = currentNode->modeFamily->getNominalPositionWorldFrame(i);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "  nominalFootPosition for leg" << i << "(region ID: " << currentNode->modeFamily->getRegionID(i) << "): " << nominalPositions[i].transpose());
        }
    }

    // get nominal torso pose
    base_coordinate_t nominalTorsoPose = getTorsoPose(nominalPositions, false);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Nominal torso pose for adding nodes: " << nominalTorsoPose.transpose());

    // get set of reachable regions for each leg
    std::vector<std::vector<int>> reachableRegionIDs(4);

    for (int i = 0; i < 4; i++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Leg " << i << " swing flag: " << swingFlags[i]);
        if (swingFlags[i])
        {
            // populate with reachable regions
            for (int j = 0; j < regions_.size(); j++)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "    Checking region " << j);
                if (superquadrics_.at(i)->isRegionWithinSuperquadricWorldFrame(nominalTorsoPose, regions_.at(j), false))
                {
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "        Region " << j << " is reachable for leg " << i);
                    // reachableRegions.at(i).push_back(regions_.at(j));
                    reachableRegionIDs.at(i).push_back(j);
                }
            }
        } else
        {
            // stance leg region
            reachableRegionIDs.at(i).push_back(currentNode->modeFamily->getRegionID(i));
        }
    }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "FL reachable regions: ");
    // for (int r : reachableRegionIDs[FL])
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "  " << r);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "FR reachable regions: ");
    // for (int r : reachableRegionIDs[FR])
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "  " << r);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "BL reachable regions: ");
    // for (int r : reachableRegionIDs[BL])
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "  " << r);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "BR reachable regions: ");
    // for (int r : reachableRegionIDs[BR])
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "  " << r);

    FLReachableCount = reachableRegionIDs[FL].size();
    FRReachableCount = reachableRegionIDs[FR].size();
    BLReachableCount = reachableRegionIDs[BL].size();
    BRReachableCount = reachableRegionIDs[BR].size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  FLReachableCount: " << FLReachableCount);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  FRReachableCount: " << FRReachableCount);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  BLReachableCount: " << BLReachableCount);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  BRReachableCount: " << BRReachableCount);

    // reachableCount = 0;
    stableStanceCount = 0;
    forwardFacingCount = 0;
    shortEnoughSwingCount = 0;
    perStepForwardProgressCount = 0;

    base_coordinate_t regionsTorsoPose;
    for (int FL_i : reachableRegionIDs[FL])
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_i: " << BL_i);
        for (int FR_i : reachableRegionIDs[FR])
        {
            if (FL_i == FR_i)
            {
                continue;
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_i: " << BR_i);
            for (int BL_i : reachableRegionIDs[BL])
            {
                if (FL_i == BL_i || FR_i == BL_i)
                {
                    continue;
                }

                // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_i: " << FL_i);
                for (int BR_i : reachableRegionIDs[BR])
                {
                    if (BR_i == BL_i || BR_i == FR_i || BR_i == FL_i)
                    {
                        continue;
                    }

                    // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_i: " << FL_i);
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_i: " << FR_i);
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_i: " << BL_i);
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_i: " << BR_i);

                    std::vector<vector3_t> nominalPositions(4);
                    // std::vector<switched_model::ConvexTerrain> footRegions(4);
                    // std::vector<int> footRegionIDs(4);
                    std::vector<int> loopRegionIDs = {FL_i, FR_i, BL_i, BR_i};

                    for (int i = 0; i < 4; i++)
                    {
                        if (swingFlags[i])
                        {
                            nominalPositions[i] = regions_[loopRegionIDs[i]].plane.positionInWorld;
                            // footRegions[i] = regions_[loopRegionIDs[i]];
                            // footRegionIDs[i] = loopRegionIDs[i];
                        } else
                        {
                            nominalPositions[i] = currentNode->modeFamily->getNominalPositionWorldFrame(i);
                            // footRegions[i] = currentNode->modeFamily->getRegion(i);
                            // footRegionIDs[i] = currentNode->modeFamily->getRegionID(i);
                        }
                    }
                    
                    bool unitTestPrint = false; // false; // 

                    regionsTorsoPose = getTorsoPose(nominalPositions, unitTestPrint);

                    // double torsoPathDeviation = calculateTorsoPathDeviation(torsoPath, nominalTorsoPose, unitTestPrint);

                    // bool isCloseEnough = true; // (torsoPathDeviation < 0.25);

                    bool stableStance = isStableStance(nominalPositions, regionsTorsoPose);

                    // if (unitTestPrint)
                    // {
                    //     RCLCPP_INFO_STREAM(node_->get_logger(), "          nominalTorsoPose: " << nominalTorsoPose.transpose());
                    // }
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "          nominalTorsoPose: " << nominalTorsoPose.transpose());                    

                    if (stableStance)
                    {
                        stableStanceCount++;

                        // bool reachable = true; // isReachable(regionsTorsoPose, footRegions, unitTestPrint);

                        // if (unitTestPrint)
                        // {
                        //     RCLCPP_INFO_STREAM(node_->get_logger(), "          reachable: " << reachable);
                        // }

                        // if (reachable)
                        // {
                        // reachableCount++;

                        // passed_reachability = true;
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "passed reachability check");

                        bool forward_facing = isForwardFacing(regionsTorsoPose);

                        if (forward_facing)
                        {
                            forwardFacingCount++;

                            // passed_forward_facing = true;
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "passed forward facing check");

                            /**
                            * Throw our transition if destination node has swing feet too far away
                            */
                            bool short_enough_swing = isSwingShortEnough(currentNode, nominalPositions);

                            if (short_enough_swing)
                            {
                                shortEnoughSwingCount++;
                                
                                // RCLCPP_INFO_STREAM(node_->get_logger(), "not throwing out transition");

                                // bool perStepForwardProgress = true; // makingPerStepForwardProgress(currentNode, nominalPositions);
                                bool forwardProgress = makingForwardProgress(nominalTorsoPose, regionsTorsoPose);

                                if (forwardProgress)
                                {
                                    perStepForwardProgressCount++;

                                    // // if FL_swing is swing, that means it should be in stance for next one
                                    // // if FL_

                                    // std::vector<int> loopRegionIDs = {-1, -1, -1, 1};
                                    // for (int i = 0; i < 4; i++)
                                    // {
                                    //     if (nextSwingFlags[i])
                                    //     {
                                    //         loopRegionIDs[i] = -1;
                                    //     } else 
                                    //     {


                                    //     }



                                    // add node  
                                    id.setRegionID(next_FL_swing ? -1 : FL_i, 
                                                    next_FR_swing ? -1 : FR_i,
                                                    next_BL_swing ? -1 : BL_i, 
                                                    next_BR_swing ? -1 : BR_i);
                                    
                                    // RCLCPP_INFO_STREAM(node_->get_logger(), "          candidate graph node ID: ");
                                    // RCLCPP_INFO_STREAM(node_->get_logger(), "              FL: " << id.FL_id_);
                                    // RCLCPP_INFO_STREAM(node_->get_logger(), "              FR: " << id.FR_id_);
                                    // RCLCPP_INFO_STREAM(node_->get_logger(), "              BL: " << id.BL_id_);
                                    // RCLCPP_INFO_STREAM(node_->get_logger(), "              BR: " << id.BR_id_);

                                    // Add node
                                    ModeFamilyNode * nextNode;
                                    if (nodeMap.count(id) == 0)
                                    {
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "          adding node ... ");
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "              FL: " << id.FL_id_);
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "              FR: " << id.FR_id_);
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "              BL: " << id.BL_id_);
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "              BR: " << id.BR_id_);

                                        //  create and add node if it doesn't exist
                                        nodeMap[id] = createNodeFromID(id, regionsTorsoPose);
                                        nodeMap[id]->setHeuristicDistance(w_euc, localGoalPose_);

                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      adding node: " << nodeMap[id]->description());
                                    } else
                                    {
                                        // already exists, we're good.
                                    }
                                    nextNode = nodeMap[id];

                                    bool transitionToPreviousNode = isTransitionToPrevious(currentNode, nextNode);

                                    if (!transitionToPreviousNode)
                                    {
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      not transitioning to previous node");
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      adding transition from " << currentNode->description() << " to " << nextNode->description());

                                        // if (unitTestPrint)
                                        // {
                                        //     RCLCPP_INFO_STREAM(node_->get_logger(), "          currentNode: " << currentNode->description());
                                        //     RCLCPP_INFO_STREAM(node_->get_logger(), "          nextNode: " << nextNode->description());
                                        // }

                                        // Add transition

                                        // bool steppingOnSameRegions = false; // isTransitionSteppingOnSameRegions(currentNode, nextNode);

                                        // if (!steppingOnSameRegions)
                                        // {
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), " not stepping on same regions");
                                        // Add transition
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      nodeI->modeFamily->getNominalTorsoPosition(): " << nodeI->modeFamily->getNominalTorsoPosition().transpose());
                                        // RCLCPP_INFO_STREAM(node_->get_logger(), "      nodeJ->modeFamily->getNominalTorsoPosition(): " << nodeJ->modeFamily->getNominalTorsoPosition().transpose());

                                        currentNode->addTransitionTo(nextNode, torsoPath, 
                                                                        w_exp, w_euc, w_stance, w_step, w_torso);
                                        // forward_edge_count += 1;
                                        edge_count++;
                                        // } else
                                        // {
                                        //     // none_edge_count += 1;
                                        // }
                                    }
                                }                
                            }
                        }
                        // }
                    }
                }
            }
        }
    }

    std::chrono::steady_clock::time_point addNodeEndTime = std::chrono::steady_clock::now();

    graph_construction_time_ += std::chrono::duration_cast<std::chrono::microseconds>(addNodeEndTime - addNodeStartTime).count();



    // iterate through all combinations of reachable regions
}

bool Graph::isTransitionToPrevious(ModeFamilyNode * currentNode, ModeFamilyNode * nextNode)
{
    if (currentNode == NULL || nextNode == NULL)
    {
        // if either node is null, we can't be transitioning to it
        throw std::runtime_error("[isTransitionToPrevious] Current or next node is null, should not happen, cannot check transition to previous node.");
    }

    if (currentNode->previousNode == NULL)
    {
        // no previous node, so we can't be transitioning to it
        return false;
    }

    // check if the next node is the previous node of the current node
    return (*currentNode->previousNode->modeFamily == *nextNode->modeFamily);
}

bool Graph::isStableStance(const std::vector<vector3_t> & nominalPositions,
                            const base_coordinate_t & torso_pose_world_frame)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[Graph::isStableStance]");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  nominalPositions: ");
    // for (int i = 0; i < nominalPositions.size(); i++)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "    leg " << i << ": " << nominalPositions[i].transpose());
    // }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  torso_pose_world_frame: " << torso_pose_world_frame.transpose());

    // large version
    // double idealStanceWidth = 0.625;
    // double idealStanceLength = 1.05;

    // small version
    // double idealStanceWidth = 0.25;
    // double idealStanceLength = 0.35;    

    // Get four vectors in world frame
    vector3_t BLtoBR_world = nominalPositions[BR] - nominalPositions[BL];
    vector3_t BRtoFR_world = nominalPositions[FR] - nominalPositions[BR];
    vector3_t FRtoFL_world = nominalPositions[FL] - nominalPositions[FR];
    vector3_t FLtoBL_world = nominalPositions[BL] - nominalPositions[FL];

    // Transform to base frame
    vector3_t p_torso_world_frame = extractTorsoPosition(torso_pose_world_frame);
    vector3_t orient_torso_world_frame = extractTorsoOrientation(torso_pose_world_frame);

    // double roll = orient_torso_world_frame[0];
    // double pitch = orient_torso_world_frame[1];
    // double yaw = orient_torso_world_frame[2];

    // Eigen::Matrix3d R_rbt2world = calculateRotationMatrix(roll, pitch, yaw);
    const Eigen::Matrix3d R_rbt2world = rotationMatrixBaseToOrigin<scalar_t>(orient_torso_world_frame);

    Eigen::Matrix3d R_world2rbt = R_rbt2world.transpose();

    vector3_t BLtoBR_base = R_world2rbt * BLtoBR_world;
    vector3_t BRtoFR_base = R_world2rbt * BRtoFR_world;
    vector3_t FRtoFL_base = R_world2rbt * FRtoFL_world;
    vector3_t FLtoBL_base = R_world2rbt * FLtoBL_world;

    // Evaluate
    // double frontWidth = FRtoFL_world.norm();
    // double backWidth = BLtoBR_world.norm();
    // double leftLength = FLtoBL_world.norm();
    // double rightLength = BRtoFR_world.norm();
    double frontWidth = std::abs(FRtoFL_base[1]);
    double backWidth = std::abs(BLtoBR_base[1]);
    double leftLength = std::abs(FLtoBL_base[0]);
    double rightLength = std::abs(BRtoFR_base[0]);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  frontWidth: " << frontWidth);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  backWidth: " << backWidth);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  leftLength: " << leftLength);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  rightLength: " << rightLength);

    double frontWidthError = std::abs(frontWidth - params_.ideal_stance_width);
    double backWidthError = std::abs(backWidth - params_.ideal_stance_width);
    double leftLengthError = std::abs(leftLength - params_.ideal_stance_length);
    double rightLengthError = std::abs(rightLength - params_.ideal_stance_length);

    // 50% deviation margin
    // Need to account for being able to step forward here
    bool stableStance = ( (frontWidthError / params_.ideal_stance_width) < 0.50 &&
                          (backWidthError / params_.ideal_stance_width) < 0.50 &&
                          (leftLengthError / params_.ideal_stance_length) < 0.50 &&
                          (rightLengthError / params_.ideal_stance_length) < 0.50 );

    return stableStance;
}

bool Graph::makingPerStepForwardProgress(const ModeFamilyNode * candidateNode, const std::vector<vector3_t> & nominalPositions)
{
    ModeFamilyNode * prevNode = candidateNode->previousNode;

    // TODO: extend to 3D 
    Eigen::Vector2d robotHeading = Eigen::Vector2d(std::cos(torsoOrientation_[2]), std::sin(torsoOrientation_[2]));

    for (int l = 0; l < 4; l++)
    {
        if (candidateNode->modeFamily->isLegInSwing(l))
        {
            vector3_t liftOffPosn = prevNode->modeFamily->getNominalPositionWorldFrame(l);
            vector3_t touchDownPosn = nominalPositions.at(l);

            vector3_t footDisplacement = touchDownPosn - liftOffPosn;
            
            // essentially adjusting foot in place
            if (footDisplacement.norm() < 0.05)
                continue;

            Eigen::Vector2d footDisplacement2D = Eigen::Vector2d(footDisplacement[0], footDisplacement[1]);
            footDisplacement2D.normalize();

            // if (footDisplacement2D.dot(robotHeading) > 0.0)
            //     continue;
            // else if (footDisplacement2D.dot(robotHeading) == 0.0)
            //     continue;
            // else
            //     return false;

            if (footDisplacement2D.dot(robotHeading) >= -0.50)
                continue;
            else
                return false;

            // if (footDisplacement[0] > 0.0)
            //     continue;
            // else if (footDisplacement[0] == 0.0)
            //     return false;
            // else
            //     return false;
        }
    }    

    return true;
}

bool Graph::makingForwardProgress(const base_coordinate_t & nominalTorsoPose, const base_coordinate_t & candidateTorsoPose)
{
    vector3_t nominalTorsoPosition = extractTorsoPosition(nominalTorsoPose);
    vector3_t candidateTorsoPosition = extractTorsoPosition(candidateTorsoPose);

    vector3_t torsoDisplacement = candidateTorsoPosition - nominalTorsoPosition;
    vector2_t torsoDisplacement2D = Eigen::Vector2d(torsoDisplacement[0], torsoDisplacement[1]);
    torsoDisplacement2D.normalize();

    vector3_t goalOrientation = extractTorsoOrientation(localGoalPose_);
    Eigen::Vector2d goalHeading = Eigen::Vector2d(std::cos(goalOrientation[2]), 
                                                    std::sin(goalOrientation[2]));

    if (torsoDisplacement2D.dot(goalHeading) > 0.0)
    {
        return true;
    } else
    {
        return false;
    }

    // // Currently, this is assuming the x-direction is forward. Pretty limited. Should dot with robot heading.
    // vector3_t torsoDisplacement = nextNode->modeFamily->getNominalTorsoPosition() - 
    //                                         candidateNode->modeFamily->getNominalTorsoPosition();

    // if (torsoDisplacement[0] > 0.0)
    //     return 1;
    // else if (torsoDisplacement[0] == 0.0)
    //     return 0;
    // else
    //     return -1;
}

bool Graph::isTransitionSteppingOnSameRegions(const ModeFamilyNode * candidateNode, const ModeFamilyNode * nextNode)
{
    bool steppingOnSameRegions = false;
    for (int legA = 0; legA < 4; legA++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "legA: " << legA);

        if (!candidateNode->modeFamily->isLegInSwing(legA)) //  && !nodeJ->modeFamily->isLegInSwing(dstLeg)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "legA is not in swing");
            for (int legB = 0; legB < 4; legB++)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "legB: " << legB);
                if (!nextNode->modeFamily->isLegInSwing(legB)
                    && candidateNode->modeFamily->twoLegsSameRegion(legA, *nextNode->modeFamily, legB))
                    steppingOnSameRegions = true;
            }
        }          

        if (!nextNode->modeFamily->isLegInSwing(legA))
        {
            for (int legB = 0; legB < 4; legB++)
            {
                if (!candidateNode->modeFamily->isLegInSwing(legB)
                    && nextNode->modeFamily->twoLegsSameRegion(legA, *candidateNode->modeFamily, legB))
                    steppingOnSameRegions = true;
            }
        }              
    }    

    return steppingOnSameRegions;
}

bool Graph::isSwingShortEnough(const ModeFamilyNode * candidateNode, const std::vector<vector3_t> & nominalPositions)
{
    ModeFamilyNode * prevNode = candidateNode->previousNode;
    bool short_enough_swing = true;
    for (int l = 0; l < 4; l++)
    {
        if (candidateNode->modeFamily->isLegInSwing(l))
        {
            vector3_t liftOffPosn = prevNode->modeFamily->getNominalPositionWorldFrame(l);
            vector3_t touchDownPosn = nominalPositions.at(l);
            double dist = (liftOffPosn - touchDownPosn).norm();
            // RCLCPP_INFO_STREAM(node_->get_logger(), "liftOffPosn: " << liftOffPosn.transpose());
            // RCLCPP_INFO_STREAM(node_->get_logger(), "touchDownPosn: " << touchDownPosn.transpose());
            // RCLCPP_INFO_STREAM(node_->get_logger(), "distance: " << (liftOffPosn - touchDownPosn).norm());
            if (dist >= params_.max_leg_swing_distance)
            {
                short_enough_swing = false;
                break;
            }
        }
    }

    return short_enough_swing;
}

bool Graph::isForwardFacing(const base_coordinate_t & nominalTorsoPose)
{
    vector3_t nominalTorsoOrientation = extractTorsoOrientation(nominalTorsoPose);

    Eigen::Vector2d nominalTorsoYawVector = Eigen::Vector2d(std::cos(nominalTorsoOrientation[2]), 
                                                            std::sin(nominalTorsoOrientation[2]));
    // Eigen::Vector2d robotHeading = Eigen::Vector2d(std::cos(torsoOrientation_[2]), 
    //                                                 std::sin(torsoOrientation_[2]));

    vector3_t goalOrientation = extractTorsoOrientation(localGoalPose_);
    Eigen::Vector2d goalHeading = Eigen::Vector2d(std::cos(goalOrientation[2]), 
                                                    std::sin(goalOrientation[2]));

    // RCLCPP_INFO_STREAM(node_->get_logger(), "nominalTorsoYawVector: " << nominalTorsoYawVector.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "robotHeading: " << robotHeading.transpose());

    return nominalTorsoYawVector.dot(goalHeading) > 0.0;
}

// double Graph::calculateTorsoPathDeviation(const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
//                                                 const base_coordinate_t & nominalTorsoPose,
//                                                 const bool & unitTestPrint)
// {
//     double min_dist = 1e30;

//     vector3_t nominalTorsoPosition = extractTorsoPosition(nominalTorsoPose);

//     double cand_dist = 0.0;
//     for (TorsoStateNode::TorsoTransition * tran : torsoPath)
//     {
//         cand_dist = (nominalTorsoPosition - tran->getNodeTo()->getPositionWorldFrame()).norm();
//         if (cand_dist < min_dist)
//             min_dist = cand_dist;
//     }

//     return min_dist;
// }

bool Graph::isReachable(const base_coordinate_t & nominalTorsoPose, 
                        const std::vector<switched_model::ConvexTerrain> & footRegions,
                        const bool & unitTestPrint)
{
    // if (unitTestPrint)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "[isReachable]");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "nominalTorsoPose: " << nominalTorsoPose.transpose());
    // }

    // double roll = nominalTorsoPose[0];
    // double pitch = nominalTorsoPose[1];
    // double yaw = nominalTorsoPose[2];

    // Eigen::Matrix3d R_rbt2world = calculateRotationMatrix(roll, pitch, yaw);
    // Eigen::Matrix3d R_world2rbt = R_rbt2world.transpose();

    // for (int i = 0; i < 4; i++)
    // {
    //     vector3_t footPosition_world = footRegions.at(i).plane.positionInWorld;
    //     vector3_t torsoPosition_world = nominalTorsoPose.tail(3);
    //     vector3_t torsoToFoot_rbt = R_world2rbt * (footPosition_world - torsoPosition_world);
    //     torsoToFoot_rbt[2] = 0.0; // project to ground plane

    //     if (unitTestPrint)
    //     {
    //         RCLCPP_INFO_STREAM(node_->get_logger(), LegNameShort.at(i));
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "footPosition_world: " << footPosition_world.transpose());
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "torsoPosition_world: " << torsoPosition_world.transpose());
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "torsoToFoot_rbt: " << torsoToFoot_rbt.transpose());
    //     }

    //     if (i == FL)
    //     {
    //         if (torsoToFoot_rbt[0] < 0.0 || torsoToFoot_rbt[1] < 0.0 || torsoToFoot_rbt.norm() > 0.30)
    //             return false;
    //     } else if (i == FR)
    //     {
    //         if (torsoToFoot_rbt[0] < 0.0 || torsoToFoot_rbt[1] > 0.0 || torsoToFoot_rbt.norm() > 0.30)
    //             return false;            
    //     } else if (i == BL)
    //     {
    //         if (torsoToFoot_rbt[0] > 0.0 || torsoToFoot_rbt[1] < 0.0 || torsoToFoot_rbt.norm() > 0.30)
    //             return false;            
    //     } else if (i == BR)
    //     {
    //         if (torsoToFoot_rbt[0] > 0.0 || torsoToFoot_rbt[1] > 0.0 || torsoToFoot_rbt.norm() > 0.30)
    //             return false;            
    //     }
    // }

    // return true;

    bool reachable = true;
    for (int i = 0; i < 4; i++)
    {
        if (!superquadrics_.at(i)->isRegionWithinSuperquadricWorldFrame(nominalTorsoPose, footRegions.at(i), unitTestPrint))
        {
            reachable = false;
            break;
        }
    }

    return reachable;
}

base_coordinate_t Graph::getTorsoPose(const std::vector<vector3_t> & regions,
                                        const bool & unitTestPrint)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  [getTorsoPose]: ");
    vector3_t avgContactPosition = (regions[FL] + regions[FR] + regions[BL] + regions[BR]) / 4.0;
    // for (int i = 0; i < 4; i++)
    //     avgContactPosition += regions[i];
    // avgContactPosition /= 4.0;

    if (unitTestPrint)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                  avgContactPosition: " << avgContactPosition.transpose());
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  avgContactPosition: " << avgContactPosition.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  FR_pair: (" << FR_pair.first.transpose() << ") --- (" << FR_pair.second.transpose() << ")");

    vector3_t localContactNormal = estimatePlaneNormal(regions);

    if (unitTestPrint)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                  localContactNormal: " << localContactNormal.transpose());
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  localContactNormal: " << localContactNormal.transpose());

    vector3_t torsoPosition = avgContactPosition + localContactNormal * Z_TORSO_OFFSET; //  = (torsoPositionI + torsoPositionJ) / 2.0;

    if (unitTestPrint)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                  torsoPosition: " << torsoPosition.transpose());
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  torsoPosition: " << torsoPosition.transpose());

    // also calculate torso orientation based on contact normal
    float referenceYaw = torsoOrientation_[2];
    vector3_t torsoOrientation = estimateEulerAnglesFromContacts(regions, referenceYaw);

    if (unitTestPrint)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                  torsoOrientation: " << torsoOrientation.transpose());
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  torsoOrientation: " << torsoOrientation.transpose());

    // Eigen::VectorXd nominalTorsoPose(6);
    base_coordinate_t nominalTorsoPose;
    nominalTorsoPose << torsoOrientation, torsoPosition;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "                  nominalTorsoPose: " << nominalTorsoPose.transpose());

    return nominalTorsoPose;
}

void Graph::addStartNode(const ModeFamilyNodeID & startStanceID, 
                            const base_coordinate_t & refTorsoPose,
                            const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath)
{
    // create node
    startNode = createNodeFromID(startStanceID, refTorsoPose);
    nodeMap[startStanceID] = startNode;
    // startNode = nodeMap[startStanceID];
    RCLCPP_INFO_STREAM(node_->get_logger(), "startNode: " << startNode->description());
    startNode->setCostToCome(0.0); // for search
    currentNode = startNode;

    ModeFamilyNodeID startSwingID(startStanceID);

    if (contactSequence->isLegInSwing(FL, 0))
        startSwingID.FL_id_ = -1;
    if (contactSequence->isLegInSwing(FR, 0))
        startSwingID.FR_id_ = -1;
    if (contactSequence->isLegInSwing(BL, 0))
        startSwingID.BL_id_ = -1;
    if (contactSequence->isLegInSwing(BR, 0))
        startSwingID.BR_id_ = -1;

    nodeMap[startSwingID] = createNodeFromID(startSwingID, refTorsoPose);

    // just make one transition to mode family where first leg is planning
    // how to actually perform this transition during planning?
    // maybe just manually override if its the first/last transition
    if (nodeMap.count(startSwingID))
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "adding start transition");
        nodeMap[startStanceID]->addTransitionTo(nodeMap[startSwingID], torsoPath,
                                                w_exp, w_euc, w_stance, w_step, w_torso);
        edge_count++;

        currentTransition = nodeMap[startStanceID]->trans.at(0);
    }

    // if (nodeMap[startStanceID]->trans.size() == 0)
    //     throw std::runtime_error("[Graph::addStartNode()]: Start node has no transitions");

}

// void Graph::addGoalNode(const vector3_t & torsoLocalWaypointPosition)
// {
//     // RCLCPP_INFO_STREAM(node_->get_logger(), "[addGoalNode()]");

//     // RCLCPP_INFO_STREAM(node_->get_logger(), "torsoLocalWaypointPosition: " << torsoLocalWaypointPosition.transpose());

//     // Leaving undefined for now, node is really more of just a wrapper for the goal position

//     ModeFamilyNodeID goalStanceID;

//     // RCLCPP_INFO_STREAM(node_->get_logger(), "pre-creating goal");
//     goalNode = createGoalNodeFromWaypoint(torsoLocalWaypointPosition);
//     // RCLCPP_INFO_STREAM(node_->get_logger(), "post-creating goal");

//     // RCLCPP_INFO_STREAM(node_->get_logger(), "pre-adding to node map");
//     nodeMap[goalStanceID] = goalNode;
//     // RCLCPP_INFO_STREAM(node_->get_logger(), "post-adding to node map");

//     // RCLCPP_INFO_STREAM(node_->get_logger(), "goalNode: " << goalNode->description());
// }

void Graph::resetGraph()
{
    contactSequence->reset();

    currentNode = startNode;

    currentTransition = NULL;
    previousTransition = NULL;

    lead_counter = 0;

    std::map<ModeFamilyNodeID, ModeFamilyNode *>::iterator nodeMapIterator;
    for (nodeMapIterator = nodeMap.begin(); nodeMapIterator != nodeMap.end(); nodeMapIterator++)        
        nodeMapIterator->second->resetNode();

    getStartNode()->setCostToCome(0.0);      
}

ModeFamilyNode * Graph::createNodeFromID(const ModeFamilyNodeID & nodeID, 
                                        const base_coordinate_t & refTorsoPose)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[createNodeFromID()]");

    switched_model::ConvexTerrain emptyRegion;

    ModeFamily * mf = new ModeFamily(node_,
                                     nodeID,
                                     nodeID.FL_id_ >= 0 ? regions_[nodeID.FL_id_] : emptyRegion, 
                                     nodeID.FR_id_ >= 0 ? regions_[nodeID.FR_id_] : emptyRegion, 
                                     nodeID.BL_id_ >= 0 ? regions_[nodeID.BL_id_] : emptyRegion, 
                                     nodeID.BR_id_ >= 0 ? regions_[nodeID.BR_id_] : emptyRegion);
    ModeFamilyNode *n = new ModeFamilyNode(*mf, refTorsoPose, search_algorithm);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "resulting node is null: " << (n == NULL));
    return n;
}

// ModeFamilyNode * Graph::createGoalNodeFromWaypoint(const vector3_t & torsoLocalWaypointPosition)
// {
//     ModeFamily * mf = new ModeFamily(torsoLocalWaypointPosition);
//     ModeFamilyNode *n = new ModeFamilyNode(*mf, search_algorithm);

//     return n;
// }

bool Graph::checkDiscreteModeSwitch()
{
    if (lead_counter == 0)
        return true;

    if (previousTransition == NULL)
        throw std::runtime_error("[checkDiscreteModeSwitch()]: prior transition is null");

    if (currentTransition == NULL)
        throw std::runtime_error("[checkDiscreteModeSwitch()]: current transition is null");

    bool discreteModeSwitch = false;

    bool sameModeFamilies = (*currentTransition->getNodeTo()->modeFamily == *previousTransition->getNodeFrom()->modeFamily);
    
    // TODO: how to only put one of two swing feet into stance if one is moving to same region?

    return sameModeFamilies;
}

// void Graph::setFromAndToRegions(const int & legIdx,
//                                 std::vector<vector3_t> & leg_from_region_points,
//                                 std::vector<vector3_t> & leg_to_region_points)
// {
//     std::vector<double> oneZeroVector = {1.0, 0.0};

//     bool foot_offset = true;
//     bool inflated_region = true;

//     // need three points to get dimensions + normal
//     if (contactSequence->isLegInSwing(legIdx))
//     {
//         leg_from_region_points.at(0) = previousTransition->getNodeFrom()->modeFamily->getMinPositionBaseFrame(legIdx, foot_offset, inflated_region);
//         leg_from_region_points.at(1) = previousTransition->getNodeFrom()->modeFamily->getRegionCoordinatesBaseFrame(legIdx, oneZeroVector, foot_offset, inflated_region);
//         leg_from_region_points.at(2) = previousTransition->getNodeFrom()->modeFamily->getMaxPositionBaseFrame(legIdx, foot_offset, inflated_region);

//         leg_to_region_points.at(0) = currentTransition->getNodeTo()->modeFamily->getMinPositionBaseFrame(legIdx, foot_offset, inflated_region);
//         leg_to_region_points.at(1) = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesBaseFrame(legIdx, oneZeroVector, foot_offset, inflated_region);
//         leg_to_region_points.at(2) = currentTransition->getNodeTo()->modeFamily->getMaxPositionBaseFrame(legIdx, foot_offset, inflated_region);
//     } else
//     {
//         // Should be the same because foot is in stance
//         leg_from_region_points.at(0) = previousTransition->getNodeTo()->modeFamily->getMinPositionBaseFrame(legIdx, foot_offset, inflated_region);
//         leg_from_region_points.at(1) = previousTransition->getNodeTo()->modeFamily->getRegionCoordinatesBaseFrame(legIdx, oneZeroVector, foot_offset, inflated_region);
//         leg_from_region_points.at(2) = previousTransition->getNodeTo()->modeFamily->getMaxPositionBaseFrame(legIdx, foot_offset, inflated_region);

//         leg_to_region_points.at(0) = currentTransition->getNodeFrom()->modeFamily->getMinPositionBaseFrame(legIdx, foot_offset, inflated_region);
//         leg_to_region_points.at(1) = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesBaseFrame(legIdx, oneZeroVector, foot_offset, inflated_region);
//         leg_to_region_points.at(2) = currentTransition->getNodeFrom()->modeFamily->getMaxPositionBaseFrame(legIdx, foot_offset, inflated_region);
//     }
// }

void Graph::advanceCounter()
{
    // don't advance for first and last, as we are not stepping there
    if (lead_counter > 0 && lead_counter < (lead.size() - 1))
        contactSequence->advanceCounter();
        
    lead_counter++;    
}

void Graph::updateCurrentTransition()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[advanceLeg]: ");

    RCLCPP_INFO_STREAM(node_->get_logger(), "  lead counter: " << lead_counter << ", lead size: " << lead.size());

    if (lead_counter < 0 || lead_counter >= lead.size())
        throw std::runtime_error("[advanceLeg]: lead counter out of bounds");
    
    if (lead_counter > 0)
        previousTransition = lead[lead_counter - 1];

    currentTransition = lead[lead_counter];

    RCLCPP_INFO_STREAM(node_->get_logger(), "  currentTransition: " << currentTransition->description());

    currentNode = currentTransition->getNodeTo();
}

}  // namespace mmp
 
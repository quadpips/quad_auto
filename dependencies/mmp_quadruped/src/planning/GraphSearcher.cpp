#include <mmp_quadruped/planning/GraphSearcher.h>


namespace mmp {

/********************************GraphSearcher******************************/

GraphSearcher::GraphSearcher(const rclcpp::Node::SharedPtr& node)
{
    node_ = node;
}

// ------ SEARCH ------------------------
//  Camera * camera,
bool GraphSearcher::search(Graph * graph, const short & costFormula,
                            const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[search()]: ");
    std::chrono::steady_clock::time_point searchTimeBegin, searchTimeEnd;

    RCLCPP_INFO_STREAM(node_->get_logger(), "Graph search with cost formula is " << (costFormula == 0 ? "Dijkstra" : costFormula == 1? "A*" : "Other"));

    // clear any old lead from previous searches
    graph->lead.clear();

    // if are we just searching over mode families, we can use the ModeFamilyNode data structure
    searchTimeBegin = std::chrono::steady_clock::now();
    ModeFamilyNode * traceNode = ModeFamilySearch(graph, costFormula, torsoPath); // camera, 
    searchTimeEnd = std::chrono::steady_clock::now();

    graph->printStats();

    if (graph->getGoalNode() == NULL) //  || *traceNode->modeFamily != *graph->getGoalNode()->modeFamily
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "[GraphSearcher::ModeFamilySearch] Goal node not found during search");
        return false;
    }

    // save the paths for whichever leg (left or right) happens first in the contact sequence.
    // traceNode = graph->getGoalNode(); // shouldn't need this if search returns goalNode, might not though for some reason.

    while (traceNode)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "tracing mode family node: " << traceNode->description() << ", node costToCome: " + std::to_string(traceNode->getCostToCome()));
        // need to turn this into a transition

        // need a vector of transitions
        if (traceNode->previousTransition)
        {
            // if (camera)
            // {
            //     // RCLCPP_INFO_STREAM(node_->get_logger(), "      transition steppability weight: ");
            //     traceNode->previousTransition->getSteppabilityWeightTerm(camera);
            // }

            RCLCPP_INFO_STREAM(node_->get_logger(), "      transition experience term:             " << traceNode->previousTransition->getModeFamilyWeight());
            RCLCPP_INFO_STREAM(node_->get_logger(), "      transition euclidean dist term:         " << traceNode->previousTransition->calculateEuclideanWeight());
            RCLCPP_INFO_STREAM(node_->get_logger(), "      transition stance deviation term:       " << traceNode->previousTransition->getStanceDeviationWeight());
            RCLCPP_INFO_STREAM(node_->get_logger(), "      transition consistency deivation term:  " << traceNode->previousTransition->getConsistencyDeviationWeight());
            RCLCPP_INFO_STREAM(node_->get_logger(), "      transition torso path deviation term:   " << traceNode->previousTransition->getTorsoPathDeviationWeight());
            // RCLCPP_INFO_STREAM(node_->get_logger(), "      transition steppability term:           " << traceNode->previousTransition->getSteppabilityWeight(camera));

            graph->lead.insert(graph->lead.begin(), traceNode->previousTransition);
        } else
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "no previous node");
        }
        traceNode = traceNode->previousNode;

        // get whatever leg is planning at the beginning of the lead, store
    }
    // }

    RCLCPP_INFO_STREAM(node_->get_logger(), "  lead (size " << graph->lead.size() << "):");

    graphSearchTime = std::chrono::duration_cast<std::chrono::microseconds>(searchTimeEnd - searchTimeBegin).count();
    RCLCPP_INFO_STREAM(node_->get_logger(), "  graph search finished in " << (1.0e-6 * graphSearchTime) << " seconds");

    RCLCPP_INFO_STREAM(node_->get_logger(), "  graph construction time: " << (1.0e-6 * graph->getGraphConstructionTime()) << " seconds");

    return true;
}

// Camera * camera,
ModeFamilyNode * GraphSearcher::ModeFamilySearch(Graph * graph, const short & costFormula,
                                                    const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[ModeFamilySearch]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "      Start node: " << graph->getStartNode()->modeFamily->description());
    RCLCPP_INFO_STREAM(node_->get_logger(), "      Goal region: " << graph->getLocalGoalRegion().transpose());

    std::chrono::steady_clock::time_point searchTimeBegin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point searchTimeCurrent;
    // int64_t updateCostToComeTime = 0;

    graph->getStartNode()->setCostToCome(0.0);

    // visited: added to the frontier, but have not been popped. Their cost may still change during the graph search.

    graph->getStartNode()->markAsVisited();

    Heap<ModeFamilyNode> h(*graph->getStartNode()); // just pass start node in as placeholder for 0-index

    ModeFamilyNode * currentNode;
    double candidateCostToCome;

    bool print = false;
    while (!h.empty())
    {
        /**
         * Pop the lowest cost node in the frontier and mark the node as explored
         */

        searchTimeCurrent = std::chrono::steady_clock::now();
        int64_t searchTimeElapsed = std::chrono::duration_cast<std::chrono::microseconds>(searchTimeCurrent - searchTimeBegin).count();
        // if ( (1.0e-6 * searchTimeElapsed) > maxSearchTime_)
        // {
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "exceeded max search time of " << maxSearchTime_ << " seconds, breaking");
        //     break;
        // }

        // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "    pre-pop");
        currentNode = h.pop();
        currentNode->markAsExplored();
        // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "    post-pop");

        /**
         * If the expanded node is the goal node, then search is done
         */
        // if (*currentNode->modeFamily == *graph->getGoalNode()->modeFamily)
        // {
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "found goal, breaking");
        //     break;
        //     // return currentNode;
        // }

        // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "  currentNode: " << currentNode->description() << ", cost: " << currentNode->getCostToCome());

        Eigen::Vector3d nominalTorsoPosition = currentNode->modeFamily->getNominalTorsoPosition();

        // RCLCPP_INFO_STREAM(node_->get_logger(), "  nominalTorsoPosition: " << nominalTorsoPosition.transpose());
        
        // check if goal has been reached
        bool goal = graph->closeEnoughToLocalGoal(nominalTorsoPosition);  // true;

        // for (int l = 0; l < 4; l++)
        // {
        //     if (currentNode->modeFamily->isLegInSwing(l))
        //     {
        //         continue;
        //         // if (currentNode->previousNode->modeFamily->getRegionID(l) != graph->getGoalNode()->modeFamily->getRegionID(l))
        //         // {
        //         //     goal = false;
        //         //     break;
        //         // }
        //     } else
        //     {
        //         // RCLCPP_INFO_STREAM(node_->get_logger(), " l: " << l << ", currentNode->modeFamily->getRegionID(l): " << currentNode->modeFamily->getRegionID(l));
        //         if (currentNode->modeFamily->getRegionID(l) != graph->getGoalNode()->modeFamily->getRegionID(l))
        //         {
        //             goal = false;
        //             break;
        //         }
        //     }
        // }

        if (goal)
        {
            // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "found goal");

            // RCLCPP_INFO_STREAM(node_->get_logger(), "adding goal node");

            // create full stance node to use at end of lead
            graph->addGoalNode(currentNode);

            graph->addTransition(currentNode, graph->getGoalNode());

            // RCLCPP_INFO_STREAM(node_->get_logger(), "added transition");

            graph->getGoalNode()->previousNode = currentNode;
            
            // RCLCPP_INFO_STREAM(node_->get_logger(), "set previous node");

            graph->getGoalNode()->previousTransition = graph->getGoalNode()->previousNode->trans[0];
            
            // RCLCPP_INFO_STREAM(node_->get_logger(), "set previous transition");

            currentNode = graph->getGoalNode();
            
            // RCLCPP_INFO_STREAM(node_->get_logger(), "set current node");
            
            break;
        }

        /**
         * Build edges (and nodes)
         */
        graph->addNodesV2(currentNode, torsoPath);
        // graph->addNodes(currentNode, torsoPath);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "  Number of transitions from currentNode: " << currentNode->trans.size());

        /**
         * Iterate through the popped node's transitions to see if we should either
         *      a. add the destination node to the frontier (if it has not been visited)
         *      b. adjust the cost of the destination node (if it has been visited and is still in the frontier)
         */
        for (ModeFamilyNode::Transition * tran : currentNode->trans)
        {
            // /**
            //  * Throw our transition if destination node has swing feet too far away
            //  */
            // ModeFamilyNode * prevNode = currentNode->previousNode;
            // ModeFamilyNode * nextNode = tran->getNodeTo();
            // bool throwout = false;
            // for (int l = 0; l < 4; l++)
            // {
            //     if (currentNode->modeFamily->isLegInSwing(l))
            //     {
            //         Eigen::Vector3d liftOffPosn = prevNode->modeFamily->getNominalPositionWorldFrame(l);
            //         Eigen::Vector3d touchDownPosn = nextNode->modeFamily->getNominalPositionWorldFrame(l);
            //         double dist = (liftOffPosn - touchDownPosn).norm();
            //         // RCLCPP_INFO_STREAM(node_->get_logger(), "liftOffPosn: " << liftOffPosn.transpose());
            //         // RCLCPP_INFO_STREAM(node_->get_logger(), "touchDownPosn: " << touchDownPosn.transpose());
            //         // RCLCPP_INFO_STREAM(node_->get_logger(), "distance: " << (liftOffPosn - touchDownPosn).norm());
            //         if (dist > 0.25)
            //         {
            //             throwout = true;
            //             break;
            //         }
            //     }
            // }

            // if (throwout)
            // {
            //     // RCLCPP_INFO_STREAM(node_->get_logger(), "throwing out transition");
            //     continue;
            // }


            // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "    candidate transition to node: " << tran->getNodeTo()->description());

            if (!tran->getNodeTo()->isExplored()) // if destination node has not already been explored
            {
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      weight: " << tran->getModeFamilyWeightTerm());
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      euclideanWeight: " << tran->getEuclideanWeightTerm());
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      stanceDeviationWeight: " << tran->getStanceDeviationWeightTerm());
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      consistencyDeviationWeight: " << tran->getConsistencyDeviationWeightTerm());
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      torsoPathDeviationWeight: " << tran->getTorsoPathDeviationWeightTerm());
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      steppabilityWeight: " << tran->getSteppabilityWeightTerm(camera));


                // tran->getSteppabilityWeightTerm(camera)
                candidateCostToCome = currentNode->getCostToCome() + (tran->getModeFamilyWeightTerm() +
                                                                        tran->getEuclideanWeightTerm() +
                                                                        tran->getStanceDeviationWeightTerm() +
                                                                        tran->getConsistencyDeviationWeightTerm() +
                                                                        tran->getTorsoPathDeviationWeightTerm());

                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      candidateCostToCome: " << candidateCostToCome);

                /**
                 * If destination node of transition has NOT been visited yet
                 */
                if (!tran->getNodeTo()->isVisited())
                {
                    // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "          nodeTo has not been visited before");
                    tran->getNodeTo()->setCostToCome(candidateCostToCome);
                    tran->getNodeTo()->previousNode = currentNode;
                    tran->getNodeTo()->previousTransition = tran;
                    tran->getNodeTo()->markAsVisited();

                    // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "          Before push");
                    h.push(*tran->getNodeTo());
                    // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "          Finished push");
                } else
                {
                    /**
                     * If the node was added earlier with greater cost (didn't get expanded yet),
                     * then we need to update the cost (remove and add back to the queue) to
                     * avoid sub-optimal and duplicate processings.
                     */
                    // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "          nodeTo has been visited before with cost: " << tran->getNodeTo()->getCostToCome());
                    if (tran->getNodeTo()->getCostToCome() > candidateCostToCome)
                    {
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "              improving costToCome");

                        // std::chrono::steady_clock::time_point updateCostToComeTimeBegin = std::chrono::steady_clock::now();
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "checking to remove modeFamilyFrontier");

                        tran->getNodeTo()->previousNode = currentNode;
                        tran->getNodeTo()->previousTransition = tran;

                        int heapIdx = h.find(*tran->getNodeTo());
                        h.updateElem(heapIdx, candidateCostToCome);
                        // if (batch == 107) RCLCPP_INFO_STREAM(node_->get_logger(), "finished update");

                        // std::chrono::steady_clock::time_point updateCostToComeTimeEnd = std::chrono::steady_clock::now();
                        // updateCostToComeTime +=
                        //     std::chrono::duration_cast<std::chrono::microseconds>(updateCostToComeTimeEnd - updateCostToComeTimeBegin).count();
                    }
                }
            }
            else
            {
                // if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "      nodeTo has been explored ");
            }
        }

    //     if (currentNode->trans.size() > 1)
    //         throw std::runtime_error("Throwing error after one transition loop");
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  updateCostToComeTime: " << updateCostToComeTime / 1.0e6 << " seconds");

    return currentNode;
}

}  // namespace mmp

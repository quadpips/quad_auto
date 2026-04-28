#include <mmp_quadruped/planning/MultiModalPlanner.h>
#include <mmp_quadruped/planning/TorsoPathPlanner.h>

namespace mmp {

MultiModalPlanner::MultiModalPlanner(const rclcpp::Node::SharedPtr& node,
                                        std::shared_ptr<MultiModalVisualizer> & multiModalVisualizer, 
                                        std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface, 
                                        const std::string & taskFile,
                                        const std::string & sqpFile,
                                        const std::string & hyperparametersFile, 
                                        const std::string & envFile,
                                        const std::string & gaitCommandFile,
                                        const std::string & gait)
{
    node_ = node;

    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner]: Initializing...");
    taskFile_ = taskFile;
    sqpFile_ = sqpFile;
    hyperparametersFile_ = hyperparametersFile;
    multiModalVisualizer_ = multiModalVisualizer;
    interface_ = interface;
    envFile_ = envFile;
    gaitCommandFile_ = gaitCommandFile;
    gait_ = gait;

    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner]: Loading hyperparameters...");

    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(hyperparametersFile_, pt);
    const std::string trials_prefix = "trials.";
    ocs2::loadData::loadPtreeValue(pt, search_algorithm, trials_prefix + "search_algorithm", verbose);

    superquadrics_.resize(4);
    superquadrics_.at(FL) = new Superquadric(node_, FL, hyperparametersFile);
    superquadrics_.at(FR) = new Superquadric(node_, FR, hyperparametersFile);
    superquadrics_.at(BL) = new Superquadric(node_, BL, hyperparametersFile);
    superquadrics_.at(BR) = new Superquadric(node_, BR, hyperparametersFile);

    // set initial latest_x, latest_u

    std::string mpc_topic_prefix = "go2";

    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner]: Loading hyperparameters...");

    // observation subscriber
    observationSub_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
        mpc_topic_prefix + "_mpc_observation", 1, 
        std::bind(&MultiModalPlanner::observationCallback, this, std::placeholders::_1));

    refTrajPublisher_ = new ReferenceTrajectoryPublisher(node_, mpc_topic_prefix, hyperparametersFile, gait_);

    // Instantiate torso path planner
    tpp_ = new TorsoPathPlanner(node_, hyperparametersFile_);

    // Instantiate constraint manager
    cm_ = new ConstraintManager(node_, interface_);

    // Instantiate graph searcher
    gs_ = new GraphSearcher(node_);

    // Instantiate graph traverser
    gt_ = new GraphTraverser(node_, cm_, interface, sqpFile_, taskFile_, gaitCommandFile_, gait_);

    // Instantiate environment processor
    ep_ = new EnvironmentProcessor(node_, interface_, envFile_, torsoInitPos, torsoGlobalGoalPose);
}

MultiModalPlanner::~MultiModalPlanner()
{
    // Clean up memory
    delete graph_;
    delete gs_;
    delete cm_;
    delete gt_;
    delete tpp_;
    delete ep_;

    // delete all superquadrics
    for (auto & sq : superquadrics_)
        delete sq;

    RCLCPP_INFO_STREAM(node_->get_logger(), "All done.");
}

void MultiModalPlanner::observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) 
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner]: Observation received.");
    std::lock_guard<std::mutex> lock(observationMutex_);

    latestObservationPtr_ = msg;

    ocs2::SystemObservation latestObservation = ocs2::ros_msg_conversions::readObservationMsg(*msg);
    comkino_state_t latest_x(latestObservation.state);

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    multiModalVisualizer_->visualizeSuperquadrics(superquadrics_, torso_pose);
}

ocs2::SystemObservation MultiModalPlanner::getLatestObservation()
{
    std::lock_guard<std::mutex> lock(observationMutex_);

    ocs2::SystemObservation latestObservation = ocs2::ros_msg_conversions::readObservationMsg(*latestObservationPtr_);
    return latestObservation;
}

bool MultiModalPlanner::readyToPlan()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[readyToPlan()]");

    // read in observation
    bool observationReceived = latestObservationPtr_ != nullptr;
    if (!observationReceived)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[MMP] Waiting for initial observation...");
    } else
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "[MMP] Initial state received: ");
        // RCLCPP_INFO_STREAM(node_->get_logger(), "           torso orientation: " << latest_x.segment(0, 3).transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "           torso position: " << latest_x.segment(3, 3).transpose());
    }

    // read in terrain
    bool terrainReceived = ep_->readyToPlan();
    if (!terrainReceived)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[MMP] Waiting for local regions...");
    } else
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[MMP] Local regions received. ");
    }

    return observationReceived && terrainReceived; // (observationReceived && stepLabelsReceived && terrainReceived);
}



bool MultiModalPlanner::runPlanner(const std::shared_ptr<ocs2::SqpSolver> & solverPtr)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner::runPlanner()]");

    // Pull off most recent observation
    ocs2::SystemObservation latestObservation = getLatestObservation();
    comkino_state_t latest_x(latestObservation.state);
    comkino_input_t latest_u(latestObservation.input);

    // transform regions
    ////////////////////////////////////////////////
    //            PROCESS ENVIRONMENT             //
    ////////////////////////////////////////////////
    //      Assuming regions and latest_q are in same frame
    ep_->extractLocalRegions(latest_x); //worldFrameToBaseFrameTransform

    tpp_->buildStateGraph(latest_x, 
                            torsoGlobalGoalPose, 
                            ep_->getMinEnvPt(),
                            ep_->getMaxEnvPt());
                            // ep_->getLocalRegions()); // 
    torsoPath = tpp_->runPlanner();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "#======================#");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "# I - EXPERIENCE PHASE #");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "#======================#");
    
    bool success = offlinePlanning(latest_x, latest_u, torsoPath, solverPtr);

    return success;
}

bool MultiModalPlanner::generatedPlan()
{
    return (!gt_->getStateTrajectoryWorldFrame().empty() && 
            !gt_->getInputTrajectoryWorldFrame().empty() && 
            !gt_->getTimeTrajectoryWorldFrame().empty());
}

void MultiModalPlanner::initializeGraph(const comkino_state_t & latest_x,
                                        const comkino_input_t & latest_u,
                                        const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                                        const int & torsoLocalWaypointIdx)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner::initializeGraph()]");

    torsoLocalWaypointPose = torsoPath[torsoLocalWaypointIdx]->getNodeTo()->getPoseWorldFrame();

    RCLCPP_INFO_STREAM(node_->get_logger(), "torsoLocalWaypointPose: " << torsoLocalWaypointPose.transpose());

    /////////////////////////////////////////////////////
    //                 INITIALIZE GRAPH                //
    /////////////////////////////////////////////////////

    // Create graph
    graph_ = new Graph(node_, latest_x, search_algorithm, gaitCommandFile_, gait_,
                        ep_->getLocalRegions(), hyperparametersFile_, 
                        torsoPath, superquadrics_);

    gt_->setGraph(graph_);
    cm_->setGraph(graph_);

    ////////////////////////////////////////////////////
    //         INITIALIZE CONSTRAINT MANAGER          //
    ////////////////////////////////////////////////////

    RCLCPP_INFO_STREAM(node_->get_logger(), "number of local regions: " << ep_->getLocalRegions().size());

    // generate start mode family and add to graph
    ModeFamilyNodeID startStanceID = cm_->getStartModeFamilyNodeID(latest_x, latest_u, ep_->getLocalRegions());
    graph_->addStartNode(startStanceID, getBasePose(latest_x), torsoPath);

    // generate goal configuration and mode family and add to graph
    // ModeFamilyNodeID goalStanceID = cm_->getGoalModeFamilyNodeID(latest_x, torsoLocalWaypointPosition, ep_->getLocalRegions());
    graph_->setLocalGoalRegion(torsoLocalWaypointPose); // goalStanceID, torsoPath 
}

bool MultiModalPlanner::offlinePlanning(const comkino_state_t & latest_x,
                                        const comkino_input_t & latest_u,
                                        const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                                        const std::shared_ptr<ocs2::SqpSolver> & solverPtr)
{
    // Extract local waypoint from global torso plan
    //      Assuming torso path and latest_q are in same frame
    int torsoLocalWaypointIdx = tpp_->extractTorsoLocalWaypointPosition(latest_x);

    ////////////////////////////////////////////////
    //      RUN INTERLEAVED SEARCH + TRAJOPT      //
    ////////////////////////////////////////////////

    bool search_res = false;
    bool res = false;

    ////////////////////////
    ///// GRAPH SEARCH /////
    ////////////////////////

    while (!search_res && torsoLocalWaypointIdx >= 0)
    {
    
        initializeGraph(latest_x, latest_u, torsoPath, torsoLocalWaypointIdx);

        search_res = gs_->search(graph_, search_algorithm, torsoPath); // camera_, 

        graph_->printGraphInfo();

        // throw std::runtime_error("Stopping after search.");

        if (!search_res)
        {
            // if search fails, try again with a different local waypoint
            torsoLocalWaypointIdx--;

            if (torsoLocalWaypointIdx < 1)
                break;

            delete graph_;
            // throw std::runtime_error("Search failed, no alternative waypoint handling implemented.");
        }
    }

    // if (!search_res)
    // {
    //     throw std::runtime_error("Search failed on all waypoints.");
    // }
    if (!search_res)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Search failed on all waypoints.");
        return false;
    }

    // if (graph_->lead.size() < 2)
    // {
    //     throw std::runtime_error("Lead is too short.");
    // }
    if (graph_->lead.size() < 2)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Lead is too short.");
        return false;
    }

    multiModalVisualizer_->publishLead(graph_->lead); // visualize leads in rviz
    multiModalVisualizer_->publishTorsoPath(torsoPath);
    torsoLocalWaypointPose = torsoPath[torsoLocalWaypointIdx]->getNodeTo()->getPoseWorldFrame();
    multiModalVisualizer_->publishTorsoLocalWaypoint(torsoLocalWaypointPose, graph_->getLocalGoalRegionTolerance());

    multiModalVisualizer_->publishTorsoInitPos(torsoInitPos, graph_->getGlobalGoalRegionTolerance());
    multiModalVisualizer_->publishTorsoGlobalGoal(torsoGlobalGoalPose, graph_->getGlobalGoalRegionTolerance());

    res = true;

    ////////////////////
    ///// TRAJ OPT /////
    ////////////////////

    // roll out lead, resolve with traj opt
    res = gt_->leadRollout(solverPtr);

    // if (res)
    // {
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory optimization successful.");
    //     failureCounter_ = 0;
    // } else
    // {
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory optimization failed.");
    //     failureCounter_++;
    //     if (failureCounter_ >= 3)
    //         throw std::runtime_error("Three consecutive planning failures, stopping.");
    // }

    // // pull off most recent transform
    // ros::Time lookupTime = ros::Time::now();
    // ros::Duration timeout = ros::Duration(1.0);
    // geometry_msgs::msg::TransformStamped baseFrameToWorldFrameTransform = 
    //     tfBuffer_.lookupTransform("world", "base_aligned", lookupTime, timeout); 

    // // transform solution from base frame to world frame
    // gt_->transformTrajectories(baseFrameToWorldFrameTransform);


    // clean up
    gt_->clearExperienceBuffers(); // memory, should do every time
    graph_->resetGraph();  
    cm_->resetConfigurations();            


    // }
    // metGlobalGoal_ = true; // just setting to true for now

    // totalTimeEnd = std::chrono::steady_clock::now();
    // double totalTime = std::chrono::duration_cast<std::chrono::microseconds>(totalTimeEnd - totalTimeBegin).count() / (1.0e6);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "Finished experience phase."); 

    return res;
}




void MultiModalPlanner::trackPlan(const std::shared_ptr<ocs2::SqpSolver> & solverPtr, const bool & success)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner::trackPlan()]");

    // prepareReferenceTrajectory(solverPtr);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "done tracking");
}

void MultiModalPlanner::prepareReferenceTrajectory(const std::shared_ptr<ocs2::SqpSolver> & solverPtr)
{
    // Think that here, we should check the flags and prepare either full or sparse reference trajectory
    // Then, populate that and pass to publish so publish is doing no logic, just sending.

    // Prepare full reference trajectory
    std::pair<ocs2::TargetTrajectories, switched_model::GaitSchedule::GaitSequence> ref_traj_pair =
                refTrajPublisher_->prepareReferenceTrajectory(gt_->getStateTrajectoryWorldFrame(), 
                                                                gt_->getInputTrajectoryWorldFrame(),
                                                                gt_->getTimeTrajectoryWorldFrame(),
                                                                gt_->getModeSchedules(),
                                                                gt_->getSwitchingTimes());     

    if (refTrajPublisher_->fullRefTraj())
    {
        // refine plan
        if (refTrajPublisher_->refineTO())
        {
            gt_->refineTO(solverPtr, ref_traj_pair.first, ref_traj_pair.second);
        }

        // multiModalVisualizer_->visualizePathNTimes(ref_traj_pair.first.stateTrajectory, 
        //                                            ref_traj_pair.first.inputTrajectory, 
        //                                             ref_traj_pair.first.timeTrajectory,
        //                                             1.0);

    } else 
    {
        ref_traj_pair.first = gt_->getSparseTargetTrajectories();

        // Prepare sparse reference trajectory
    }

    latestReferenceTrajectory_ = ref_traj_pair.first;
    latestGaitSequence_ = ref_traj_pair.second;

    return;
}

void MultiModalPlanner::publishReferenceTrajectory()
{
    refTrajPublisher_->sendReferenceTrajectory(latestReferenceTrajectory_, latestGaitSequence_);
}

// Phase I - event-based, only trigger a re-plan if tracking fails.
//  - how to signal that reference trajectory is still being tracked so robot knows to 
//  - not start re-planning?
//  - Or, is it fine to just start re-planning
void MultiModalPlanner::monitorReferenceTrajectory()
{
    // Flag labels:
    //      -1 --- still tracking
    //       0 --- tracking interrupted but robot still on path, need a re-plan
    //       1 --- tracking failed (robot slipped off contacts)
    //       2 --- tracking succeeded, need a re-plan    
    int flag = -1;
    bool metLocalGoal_ = false;

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && flag < 0)
    {
        // if (cameraOn_)
        // {
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "      checking steppability during tracking");

        //     // update robot's current state
        //     Eigen::VectorXd current_state = latestObservation_.state;
        //     latest_q = current_state.tail<18>();

        //     //////////////////////////////////////////
        //     // CHECK STEPPABILITY LABELS ALONG LEAD //
        //     //////////////////////////////////////////

        //     // find closest mode to where robot currently is within lead 
        //     int min_dist_idx = 0;
        //     double min_dist = 1e30;

        //     for (int lead_counter = 0; lead_counter < graph_->lead.size(); lead_counter++)
        //     {
        //         double dist = (graph_->lead[lead_counter]->calculateNominalTorsoPositionBaseFrame() - latest_q.head(3)).norm();
        //         if (dist < min_dist)
        //         {
        //             min_dist = dist;
        //             min_dist_idx = lead_counter;
        //         }
        //     }

        //     RCLCPP_INFO_STREAM(node_->get_logger(), "      closest step counter: " << min_dist_idx);

        //     // Evaluate steppability label of nearby contact modes
        //     while (min_dist_idx < graph_->lead.size())                    
        //     {
        //         RCLCPP_INFO_STREAM(node_->get_logger(), "      checking step: " << min_dist_idx);

        //         int label = graph_->lead[min_dist_idx]->getSteppabilityLabel(camera_);

        //         RCLCPP_INFO_STREAM(node_->get_logger(), "      label: " << label);

        //         if (label == OOB) // label out of bounds
        //         {
        //             min_dist_idx++; // go to next step
        //             continue;
        //         }

        //         stepLabelBuffer_.push_back(label); // push label into step label buffer
        //         break;
        //     }

        //     // loop through past K labels, if all K are not steppable, then stop tracking
        //     bool stopTracking = true;
        //     for (boost::circular_buffer<int>::iterator it = stepLabelBuffer_.begin(); it != stepLabelBuffer_.end(); it++)
        //     {        
        //         if (*it == STEP)
        //         {
        //             stopTracking = false;
        //             break;
        //         }
        //     }

        //     if (stopTracking)
        //     {
        //         flag = 0;
        //         refTrajPublisher_->stopTracking(latestObservation_.time, 
        //                                         latestObservation_.state, 
        //                                         latestObservation_.input);
        //         RCLCPP_INFO_STREAM(node_->get_logger(), "          TRACKING FAILED - FOOTHOLD LABEL NOT STEPPABLE");
                
        //         ros::Duration(2.5).sleep(); // wait for few seconds to allow robot to stop before starting a  re-plan 
        //     }
        // }

        ////////////////////////////////
        // CHECK IF STATE IS UNSTABLE //
        ////////////////////////////////
        // if roll is too big
        // momentum is too big
        // velocity is too big

        ///////////////////////////////////////////
        // CHECK IF FINAL MODE CONSTRAINT IS MET //
        ///////////////////////////////////////////

        // RCLCPP_INFO_STREAM(node_->get_logger(), "      checking final mode constraint ");

        if (refTrajPublisher_->isTrackingDoneStateBased())
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "          tracking is done!");
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.5);
            rclcpp::sleep_for(std::chrono::nanoseconds(duration.nanoseconds())); // wait for few seconds to allow robot to stop before starting a  re-plan

            // Pull off most recent observation
            ocs2::SystemObservation latestObservation = getLatestObservation();
            comkino_state_t latest_x = latestObservation.state;

            metLocalGoal_ = cm_->checkGoalModeFamilyConstraint(latest_x);

            if (metLocalGoal_) // check proximity to local goal
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "              local goal mode family constraint IS met");
                flag = 2; // if satisfying final constraint --> 2
            } else 
            {
                flag = 1; // if not --> 1
                RCLCPP_INFO_STREAM(node_->get_logger(), "              local goal mode family constraint IS NOT met, continuing on ... ");
                // throw std::runtime_error(" local goal mode family constraint IS NOT met");
            }
        }    

        rclcpp::spin_some(node_);
        rate.sleep();
    }
}

void MultiModalPlanner::checkGlobalGoalStatus()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner::checkGlobalGoalStatus()]");
    // Pull off most recent observation
    ocs2::SystemObservation latestObservation = getLatestObservation();
    comkino_state_t latest_x(latestObservation.state);
    // global goal check
    metGlobalGoal_ = cm_->checkGlobalGoalConstraint(latest_x, torsoGlobalGoalPose);  
}

void MultiModalPlanner::clear()
{
    // std::lock_guard<std::mutex> lock(observationMutex_);

    RCLCPP_INFO_STREAM(node_->get_logger(), "[MultiModalPlanner::clear()]");
    // memory allocation
    // gm_->wipeGraph(); // delete old graph
    delete graph_;
    // refTrajPublisher_->clearReferenceTrajectory(); // clear old reference trajectory
    // gm_->clearLead(); // clear old lead

    // reset readyToPlan
    // latest_x = Eigen::VectorXd::Zero(0);
    // latest_u = Eigen::VectorXd::Zero(0);
    latestObservationPtr_ = nullptr; // reset latest observation pointer

    ep_->clearRegions();

    RCLCPP_INFO_STREAM(node_->get_logger(), "done clearing");
}

}  // namespace mmp


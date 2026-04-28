#include <mmp_quadruped/planning/GraphTraverser.h>


namespace mmp {

GraphTraverser::GraphTraverser(const rclcpp::Node::SharedPtr& node,
                                ConstraintManager * cm, 
                                std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface,
                                const std::string & sqpFile,
                                const std::string & taskFile,
                                const std::string & gaitCommandFile,
                                const std::string & gait)
{
    node_ = node;
    constraintManager_ = cm;
    interface_ = interface;

    gaitCommandFile_ = gaitCommandFile;
    gait_ = gait;

    costBuckets = {0, 1, 2, 5, 10, 50, 100, 200, 300,
                   400, 500, 600, 700, 800, 900, 1000, std::numeric_limits<int>::infinity()};
    costCounts.resize(costBuckets.size());

    iterBuckets = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    iterCounts.resize(iterBuckets.size());    

    sqpSettings = ocs2::sqp::loadSettings(sqpFile); // const ocs2::log::Settings 

    // this NEEDS to be run in this file, don't know why.
    ocs2::loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeHorizon_);
}

void GraphTraverser::setUpInterface()
{ 
    // ====== Set the scenario to the correct interfaces ========

    // ====== Gait ===== //
    ocs2::ModeSchedule modeSchedule = switched_model::loadModeScheduleFromGait(gaitCommandFile_,
                                                                                gait_,
                                                                                graph_->contactSequence->getSwingPhaseCounter(),
                                                                                false);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "updating to mode schedule: ");
    // RCLCPP_INFO_STREAM(node_->get_logger(), modeSchedule);

    //   RCLCPP_INFO_STREAM(node_->get_logger(), "querying eventTimes: ");
    //   for (int i = 0; i < eventTimes.size(); i++)
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "    [" << i << "]: " << eventTimes[i]);

    initialSwingTime_ = modeSchedule.getInitialEventTime();
    finalSwingTime_ = modeSchedule.getFinalEventTime();

    interface_->getSwitchedModelModeScheduleManagerPtr()->setModeSchedule(modeSchedule);

    // ===== Terrain ===== //
    
}

// void GraphTraverser::setUpInterface(const scalar_t & timeHorizon, 
//                                     const scalar_t & initialSwingTime, 
//                                     const scalar_t & finalSwingTime,
//                                     std::vector<std::vector<vector3_t>> & from_region_points,
//                                     std::vector<std::vector<vector3_t>> & to_region_points)
// {
//     // CHANGE INTERFACE'S GAIT SCHEDULE AND MAKE NEW referenceManagerPtr
//     // REDO PART OF OPTIMAL CONTROL PROBLEM SETUP

//     interface_->updateModeSchedule(graph_->contactSequence->getSwingPhaseCounter()); // interface.updateModeSchedule(planningFootModeSchedule);

//     // for (int leg = 0; leg < 4; leg++)
//     //     graph_->setFromAndToRegions(FL, from_region_points.at(leg), to_region_points.at(leg));

//     interface_->updateSteppableRegions(from_region_points, to_region_points);

//     interface_->setMaxSwingTime(finalSwingTime - initialSwingTime);

//     interface_->updateTorsoPosition(constraintManager_->getCurrentState().head(3));
//     interface_->updateOptimalControlProblem();
// }

bool GraphTraverser::runSQP(std::vector<ExperienceStep *> &path,
                            const std::shared_ptr<ocs2::SqpSolver> & solverPtr)
{
    // Initial state
    comkino_state_t init_sqp_cent_state = comkino_state_t::Zero();
    comkino_input_t init_sqp_cent_input = comkino_input_t::Zero();

    // Target state
    comkino_state_t target_sqp_cent_state = comkino_state_t::Zero();
    comkino_input_t target_sqp_cent_input = comkino_input_t::Zero();

    // Failed transition state
    comkino_state_t failed_transition_state =  comkino_state_t::Zero();

    // clear any old trajectories from prior traj opt runs
    clearTrajectories();

    sparseCounter_ = 0;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "PLANNING START");
    while (!graph_->reachedGoal())
    {   
        ExperienceStep * es = new ExperienceStep(graph_->getCurrentTransition(), graph_->lead);

        // Update constraints from bar positions of all 4 legs from current state in the graphs
        bool foundTransition = constraintManager_->findTransitionConfiguration(); // alreadySatisfyingConstraint updates current and target

        // if IKProjection iteration = 0, already satisfying constraint.
        if (!graph_->checkDiscreteModeSwitch())
        {
            if (!foundTransition)
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "Could not find transition configuration, deeming infeasible");
                es->setResult("Failed to find transition");
                es->setTrajOptCost(500.0); // 1e30;
                path.push_back(es);

                // create failed step out of target
                failed_transition_state = constraintManager_->getTargetState();

                // make a trajectory of failed transition to visualize
                for (int i = 0; i < 100; i++)
                {
                    double t = i * 0.01;
                    // failed_step_state_trajectory_base_frame.push_back(failed_transition_state);
                    // failed_step_input_trajectory_base_frame.push_back(init_sqp_cent_input);
                    // failed_step_time_trajectory_base_frame.push_back(t);
                    failed_step_state_trajectory_world_frame.push_back(failed_transition_state);
                    failed_step_input_trajectory_world_frame.push_back(init_sqp_cent_input);
                    failed_step_time_trajectory_world_frame.push_back(t);
                }

                return false;
            }

            RCLCPP_INFO_STREAM(node_->get_logger(), constraintManager_->descriptionPositions()); // prints target step locations

            // plug start/target into trajopt
            init_sqp_cent_state = constraintManager_->getCurrentState();
            target_sqp_cent_state = constraintManager_->getTargetState();
            init_sqp_cent_input = constraintManager_->getCurrentInput();
            target_sqp_cent_input = constraintManager_->getCurrentInput();

            solverPtr->reset();

            // Set up interface
            setUpInterface();
            // setUpInterface(timeHorizon, initialSwingTime, finalSwingTime, from_region_points, to_region_points);

            // scalar_t initialSwingTime = interface_->getSwitchedModelModeScheduleManagerPtr()->getInitialSwingTime();
            // // scalar_t finalSwingTime = interface_->getSwitchedModelModeScheduleManagerPtr()->getFinalSwingTime();
            // // scalar_t initialSwingTime = 0.05;
            // scalar_t finalSwingTime = 0.45;

            RCLCPP_INFO_STREAM(node_->get_logger(), "initialSwingTime_: " << initialSwingTime_);
            RCLCPP_INFO_STREAM(node_->get_logger(), "finalSwingTime_: " << finalSwingTime_);


            // // SqpSolver
            // ocs2::SqpSolver solver(sqpSettings, interface_->getOptimalControlProblem(), interface_->getInitializer());
            // solver.setReferenceManager(interface_->getReferenceManagerPtr());
            // solver.setSynchronizedModules(interface_->getSynchronizedModules());
            // SqpSolver set initial guess 
            if (graph_->getCurrentTransition()->getTraversalCount() > 0)
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "Yes Warm-starting!");
                // set initial guess exists
                solverPtr->setInitialGuess(graph_->getCurrentTransition()->getBestSolution());
            } else
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "Not warm-starting!");
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(), "target_sqp_cent_state: " << target_sqp_cent_state.transpose());

            RCLCPP_INFO_STREAM(node_->get_logger(), "init_sqp_cent_state: ");
            RCLCPP_INFO_STREAM(node_->get_logger(), stateString(init_sqp_cent_state));

            RCLCPP_INFO_STREAM(node_->get_logger(), "target_sqp_cent_state: ");
            RCLCPP_INFO_STREAM(node_->get_logger(), stateString(target_sqp_cent_state));

            if (std::abs(timeHorizon_ - 
                            (graph_->contactSequence->getStanceTime() + graph_->contactSequence->getSwingTime())) > 1.0e-6)
                throw std::runtime_error("timeHorizon not equal to stanceTime + swingTime. ");

            // RCLCPP_INFO_STREAM(node_->get_logger(), "pre-setTargetTrajectories");
            ocs2::TargetTrajectories targetTrajectory = ocs2::TargetTrajectories({initialSwingTime_, finalSwingTime_}, 
                                                                                {init_sqp_cent_state, target_sqp_cent_state}, 
                                                                                {init_sqp_cent_input, target_sqp_cent_input});
            solverPtr->getReferenceManager().setTargetTrajectories(targetTrajectory);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "post-setTargetTrajectories");
            

            // run trajopt
            solverPtr->run(0.0, init_sqp_cent_state, timeHorizon_);
            es->setSingleModePlanningTime(1.0e-3 * solverPtr->getTotalTimeInMilliseconds()); // want to store in seconds
            RCLCPP_INFO_STREAM(node_->get_logger(), "finished plan, took " << es->getSingleModePlanningTime() << " seconds");

            const auto primalSolution = solverPtr->primalSolution(timeHorizon_);
            const auto performance = solverPtr->getPerformanceIndeces();
            const size_t numIterations = solverPtr->getNumIterations();

            // RCLCPP_INFO_STREAM(node_->get_logger(), "final state is: " << primalSolution.stateTrajectory_.back().transpose());
            RCLCPP_INFO_STREAM(node_->get_logger(), "------performance index------");
            RCLCPP_INFO_STREAM(node_->get_logger(), performance);

            // update current state to last state from solution
            // RCLCPP_INFO_STREAM(node_->get_logger(), "actual end torso position: " << primalSolution.stateTrajectory_.back().segment(3,3).transpose());
            // RCLCPP_INFO_STREAM(node_->get_logger(), "actual sqp state: " << primalSolution.stateTrajectory_.back().transpose());
            // RCLCPP_INFO_STREAM(node_->get_logger(), "middle input: " << primalSolution.inputTrajectory_[int(primalSolution.inputTrajectory_.size() / 2.)].transpose());

            RCLCPP_INFO_STREAM(node_->get_logger(), "final state: ");
            RCLCPP_INFO_STREAM(node_->get_logger(), stateString(primalSolution.stateTrajectory_.back()));

            constraintManager_->setCurrentState(primalSolution.stateTrajectory_.back());
            constraintManager_->setCurrentInput(primalSolution.inputTrajectory_.back());

            es->setTrajOptCost(performance.cost);
            es->setNumIterations(numIterations);
            es->setSolution(primalSolution);

            // Extract collision avoidance constraint information
            //      1. Extract cost of collision avoidance constraint
            //      2. Extract raw constraint value 
            //      3. Encode as hard inequality constraint
            bool collision = false;

            // for (int i = 0; i < solver.getSolutionMetrics().intermediates.size(); i++) // iter through vector<Metrics>
            // {
            //     // RCLCPP_INFO_STREAM(node_->get_logger(), "i: " << i << " --- state soft constraint: " << solver.getSolutionMetrics().intermediates[i].stateSoftConstraint);
            //     if (solver.getSolutionMetrics().intermediates[i].stateSoftConstraint < 0.0)
            //         collision = true;
            // }   

            recordSQPStats(es);

            // appendToTerrainProfile(primalSolution, from_region_points, to_region_points);

            appendToReferenceTrajectory(primalSolution, initialSwingTime_, finalSwingTime_);
            appendToSparseReferenceTrajectory(targetTrajectory);

            // make sure feet at end of trajopt are on desired regions
            if (!constraintManager_->checkDestinationModeFamilyConstraint())
            {
                es->setTrajOptCost(500.0);
                RCLCPP_INFO_STREAM(node_->get_logger(), "dest mode family constraint not met");
                es->setResult("dest mode family constraint not met");
                path.push_back(es);
                addFailedStep(primalSolution);

                mode_schedules.push_back("STANCE");

                return true;
            } else if (collision)
            {
                es->setTrajOptCost(500.0);
                RCLCPP_INFO_STREAM(node_->get_logger(), "collision detected, returning");
                es->setResult("collision detected");
                path.push_back(es);
                addFailedStep(primalSolution);

                return false;
            } else if (performance.cost > 250.0)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "Maxed out on iterations, returning");
                // es->result = "Max iterations";
                RCLCPP_INFO_STREAM(node_->get_logger(), "Step cost too high, returning");
                es->setResult("Step cost too high");
                path.push_back(es);
                addFailedStep(primalSolution);

                return false;
            } else
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "Found solution");
                es->setResult("Found solution");
                path.push_back(es);   
            }

        } else 
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Mode switch: no step");
            es->setResult("Mode switch: no step");
            es->setTrajOptCost(0.0);
            path.push_back(es);
        }
        
        graph_->advanceCounter();

        // Grab current transition
        graph_->updateCurrentTransition();
    }

    // push back one final stance phase
    RCLCPP_INFO_STREAM(node_->get_logger(), "adding last stance phase");
    mode_schedules.push_back("STANCE");


    // RCLCPP_INFO_STREAM(node_->get_logger(), "finished while loop");

    return true; 
}

void GraphTraverser::appendToSparseReferenceTrajectory(const ocs2::TargetTrajectories & targetTrajectories)
{
    ocs2::TargetTrajectories updatedTargetTrajectories = targetTrajectories;
    // update time
    for (auto & time : updatedTargetTrajectories.timeTrajectory)
    {
        time += sparseCounter_ * timeHorizon_;
    }
    sparseCounter_++;

    sparseTargetTrajectories_.timeTrajectory.insert(sparseTargetTrajectories_.timeTrajectory.end(),
                                                    updatedTargetTrajectories.timeTrajectory.begin(),
                                                    updatedTargetTrajectories.timeTrajectory.end());
    sparseTargetTrajectories_.stateTrajectory.insert(sparseTargetTrajectories_.stateTrajectory.end(),
                                                    updatedTargetTrajectories.stateTrajectory.begin(),
                                                    updatedTargetTrajectories.stateTrajectory.end());
    sparseTargetTrajectories_.inputTrajectory.insert(sparseTargetTrajectories_.inputTrajectory.end(),
                                                    updatedTargetTrajectories.inputTrajectory.begin(),
                                                    updatedTargetTrajectories.inputTrajectory.end());
}

void GraphTraverser::appendToReferenceTrajectory(const ocs2::PrimalSolution & primalSolution,
                                                    const scalar_t & initialSwingTime,
                                                    const scalar_t & finalSwingTime)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[appendToReferenceTrajectory()]");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      appending the following solution...");
    // for (int i = 0; i < primalSolution.stateTrajectory_.size(); i++)
    // {
    //     // std::vector<vector3_t> foot_positions = constraintManager_->getFootholds(primalSolution.stateTrajectory_[i]);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "          time: " << primalSolution.timeTrajectory_[i]);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "          state: ");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "              full state: " << primalSolution.stateTrajectory_[i].transpose());
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "              centroidal state: " << primalSolution.stateTrajectory_[i].segment(0, 6).transpose());
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "              torso state: " << primalSolution.stateTrajectory_[i].segment(6, 6).transpose());
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "              legs state: " << primalSolution.stateTrajectory_[i].segment(12, 12).transpose());
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "          foot positions: ");
    //     // for (int j = 0; j < 4; j++)
    //         // RCLCPP_INFO_STREAM(node_->get_logger(), "              " << LegNameShort.at(j) << ": " << foot_positions[j].transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "          input: ");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "              full input: " << primalSolution.inputTrajectory_[i].transpose());
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "              foot contact forces: " << primalSolution.inputTrajectory_[i].segment(0, 12).transpose());
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "              foot velocities wrt base: " << primalSolution.inputTrajectory_[i].segment(12, 12).transpose());
    // }

    state_trajectory_world_frame.insert(state_trajectory_world_frame.end(), 
                                        primalSolution.stateTrajectory_.begin(), 
                                        primalSolution.stateTrajectory_.end()); 
    input_trajectory_world_frame.insert(input_trajectory_world_frame.end(), 
                                        primalSolution.inputTrajectory_.begin(), 
                                        primalSolution.inputTrajectory_.end());   
    time_trajectory_world_frame.insert(time_trajectory_world_frame.end(), 
                                        primalSolution.timeTrajectory_.begin(), 
                                        primalSolution.timeTrajectory_.end());

    // state_trajectory_base_frame.insert(state_trajectory_base_frame.end(), 
    //                                     primalSolution.stateTrajectory_.begin(), 
    //                                     primalSolution.stateTrajectory_.end()); 
    // input_trajectory_base_frame.insert(input_trajectory_base_frame.end(), 
    //                                     primalSolution.inputTrajectory_.begin(), 
    //                                     primalSolution.inputTrajectory_.end());   
    // time_trajectory_base_frame.insert(time_trajectory_base_frame.end(), 
    //                                     primalSolution.timeTrajectory_.begin(), 
    //                                     primalSolution.timeTrajectory_.end());

    mode_schedules.push_back("STANCE");
    mode_schedules.push_back(graph_->contactSequence->getCurrentMode());
    // mode_schedules.push_back(graph_->contactSequence->getNextMode());

    RCLCPP_INFO_STREAM(node_->get_logger(), "      appending the following mode schedules...");
    RCLCPP_INFO_STREAM(node_->get_logger(), "          mode 1: " << mode_schedules[mode_schedules.size() - 2]);
    RCLCPP_INFO_STREAM(node_->get_logger(), "          mode 2: " << mode_schedules[mode_schedules.size() - 1]);


    switching_times.push_back(std::to_string(initialSwingTime));
    switching_times.push_back(std::to_string(finalSwingTime));

    RCLCPP_INFO_STREAM(node_->get_logger(), "      appending the following switching times...");
    RCLCPP_INFO_STREAM(node_->get_logger(), "          time 1: " << switching_times[switching_times.size() - 2] );
    RCLCPP_INFO_STREAM(node_->get_logger(), "          time 2: " << switching_times[switching_times.size() - 1]);
}

// void GraphTraverser::appendToTerrainProfile(const ocs2::PrimalSolution & primalSolution,
//                                             const std::vector<std::vector<vector3_t>> & from_region_points,
//                                             const std::vector<std::vector<vector3_t>> & to_region_points)
// {
//     RCLCPP_INFO_STREAM(node_->get_logger(), "[appendToTerrainProfile()]");

//     // get mid swing time
//     scalar_t startSwingTime = interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getInitialSwingTime();
//     scalar_t endSwingTime = interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getFinalSwingTime();
//     scalar_t midTime = (startSwingTime + endSwingTime) / 2.0;

//     for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++)
//     {
//         Eigen::VectorXd terrainNormals = Eigen::VectorXd::Zero(12);
//         scalar_t t_curr = primalSolution.timeTrajectory_[i];
//         if (t_curr <= midTime)
//         {
//             // get from orientation
//             for (int leg = 0; leg < 4; leg++)
//                 terrainNormals.segment(3 * leg, 3) = calculateOrientationFromRegionPoints(from_region_points[leg]);
//         } else
//         {
//             // get to orientation
//             for (int leg = 0; leg < 4; leg++)
//                 terrainNormals.segment(3 * leg, 3) = calculateOrientationFromRegionPoints(to_region_points[leg]);            
//         }

//         RCLCPP_INFO_STREAM(node_->get_logger(), "      t_curr: " << t_curr << ", terrainNormals: " << terrainNormals.transpose());

//         terrain_normals_trajectory_base_frame.push_back(terrainNormals);
//     }
// }

void GraphTraverser::refineTO(const std::shared_ptr<ocs2::SqpSolver> & solverPtr,
                                ocs2::TargetTrajectories & targetTrajectories,
                                const GaitSchedule::GaitSequence & gaitSequence)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[refineTO]:");

    solverPtr->reset();

    // run trajopt
    solverPtr->getReferenceManager().setTargetTrajectories(targetTrajectories);
    
    const comkino_state_t init_sqp_cent_state = targetTrajectories.stateTrajectory.front();
    const scalar_t timeHorizon = targetTrajectories.timeTrajectory.back();

    // set up interface
    // Load mode schedule / gait schedule?

    // GaitSchedule should be the same
    ocs2::ModeSchedule modeSchedule = switched_model::getModeSchedule(0, 0.0, timeHorizon, gaitSequence.begin(), gaitSequence.end());
    interface_->getSwitchedModelModeScheduleManagerPtr()->setModeSchedule(modeSchedule);

    solverPtr->run(0.0, init_sqp_cent_state, timeHorizon);
    RCLCPP_INFO_STREAM(node_->get_logger(), "finished refinement, took " << ( 1.0e-3 * solverPtr->getTotalTimeInMilliseconds() ) << " seconds");

    const auto primalSolution = solverPtr->primalSolution(timeHorizon_);

    refinedTargetTrajectories_.timeTrajectory = primalSolution.timeTrajectory_;
    refinedTargetTrajectories_.stateTrajectory = primalSolution.stateTrajectory_;
    refinedTargetTrajectories_.inputTrajectory = primalSolution.inputTrajectory_;
    refinedGaitSequence_ = gaitSequence;

    targetTrajectories.timeTrajectory = primalSolution.timeTrajectory_;
    targetTrajectories.stateTrajectory = primalSolution.stateTrajectory_;
    targetTrajectories.inputTrajectory = primalSolution.inputTrajectory_;
    // do not need to update gait sequence since it is the same
}

void GraphTraverser::recordSQPStats(ExperienceStep * es)
{
    // record cost distribution
    for (int i = 0; i < (costBuckets.size() - 1); i++)
    {
        if (costBuckets[i] < es->getTrajOptCost() && es->getTrajOptCost() <= costBuckets[i + 1])
        {
            costCounts[i] = (costCounts[i] + 1);
            break;
        }
    }

    // record iteration distribution
    for (int i = 0; i < (iterBuckets.size() - 1); i++)
    {
        if (iterBuckets[i] < es->getNumIterations() && es->getNumIterations() <= iterBuckets[i + 1])
        {
            iterCounts[i] = (iterCounts[i] + 1);
            break;
        }
    }
}

void GraphTraverser::addFailedStep(const ocs2::PrimalSolution & sqpSolution)
{
    failed_step_state_trajectory_base_frame.insert(failed_step_state_trajectory_base_frame.end(), 
                                                    sqpSolution.stateTrajectory_.begin(), 
                                                    sqpSolution.stateTrajectory_.end()); 
    
    failed_step_input_trajectory_base_frame.insert(failed_step_input_trajectory_base_frame.end(),
                                                    sqpSolution.inputTrajectory_.begin(), 
                                                    sqpSolution.inputTrajectory_.end());
    
    failed_step_time_trajectory_base_frame.insert(failed_step_time_trajectory_base_frame.end(),
                                                    sqpSolution.timeTrajectory_.begin(), 
                                                    sqpSolution.timeTrajectory_.end());

    failed_step_state_trajectory_world_frame.insert(failed_step_state_trajectory_world_frame.end(), 
                                                    sqpSolution.stateTrajectory_.begin(), 
                                                    sqpSolution.stateTrajectory_.end()); 
    
    failed_step_input_trajectory_world_frame.insert(failed_step_input_trajectory_world_frame.end(),
                                                    sqpSolution.inputTrajectory_.begin(), 
                                                    sqpSolution.inputTrajectory_.end());
    
    failed_step_time_trajectory_world_frame.insert(failed_step_time_trajectory_world_frame.end(),
                                                    sqpSolution.timeTrajectory_.begin(), 
                                                    sqpSolution.timeTrajectory_.end());
}

bool GraphTraverser::leadRollout(const std::shared_ptr<ocs2::SqpSolver> & solverPtr)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[leadRollout]:");
    std::vector<ExperienceStep *> path;

    RCLCPP_INFO_STREAM(node_->get_logger(), "#===========================running SQP=================================#");

    bool success = runSQP(path, solverPtr);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Finished trial");

    // if (success)
    // {
    //     // Refine long horizon TO
    //     refineTO(path, solverPtr);
    // }

    experienceWalk = path;

    RCLCPP_INFO_STREAM(node_->get_logger(), "#=======================================================================#");

    return success;
}

// void GraphTraverser::transformTrajectories(const geometry_msgs::msg::TransformStamped & baseFrameToWorldFrameTransform)
// {
//     RCLCPP_INFO_STREAM(node_->get_logger(), "[transformTrajectories()]");

//     // RCLCPP_INFO_STREAM(node_->get_logger(), "      state_trajectory_base_frame.size(): " << state_trajectory_base_frame.size());
//     // RCLCPP_INFO_STREAM(node_->get_logger(), "      state_trajectory_base_frame[0].size(): " << state_trajectory_base_frame[0].size());

//     // transform stateTrajectory
//     state_trajectory_world_frame.resize(state_trajectory_base_frame.size());
//     for (int i = 0; i < state_trajectory_base_frame.size(); i++)
//     {
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "      state " << i);

//         Eigen::VectorXd state_i_base_frame = state_trajectory_base_frame[i];
//         Eigen::VectorXd state_i_world_frame = Eigen::VectorXd::Zero(state_i_base_frame.size());

//         // RCLCPP_INFO_STREAM(node_->get_logger(), "      state_i_base_frame: ");
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          centroidal state: " << state_i_base_frame.segment(0, 6).transpose());
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          torso state: " << state_i_base_frame.segment(6, 6).transpose());
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          legs state: " << state_i_base_frame.segment(12, 12).transpose());

//         // first 6 (centroidal state, transform vector)
//         state_i_world_frame.segment(0, 3) = transformHelperVector3Stamped(state_i_base_frame.segment(0, 3),
//                                                                             baseFrameToWorldFrameTransform);
//         state_i_world_frame.segment(3, 3) = transformHelperVector3Stamped(state_i_base_frame.segment(3, 3),
//                                                                             baseFrameToWorldFrameTransform);

//         // next 6 (torso state, transform pose)
//         state_i_world_frame.segment(6, 6) = transformHelperPoseStamped(state_i_base_frame.segment(6, 6), 
//                                                                         baseFrameToWorldFrameTransform);

//         // last 12 (joint states, no transform needed)
//         state_i_world_frame.tail<12>() = state_i_base_frame.tail<12>();

//         // RCLCPP_INFO_STREAM(node_->get_logger(), "      state_i_world_frame " << state_i_world_frame.transpose());

//         // RCLCPP_INFO_STREAM(node_->get_logger(), "      state_i_world_frame: ");
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          centroidal state: " << state_i_world_frame.segment(0, 6).transpose());
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          torso state: " << state_i_world_frame.segment(6, 6).transpose());
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          legs state: " << state_i_world_frame.segment(12, 12).transpose());

//         // setting
//         state_trajectory_world_frame[i] = state_i_world_frame;
//     }

//     // transform inputTrajetory
//     input_trajectory_world_frame.resize(input_trajectory_base_frame.size());
//     for (int i = 0; i < input_trajectory_base_frame.size(); i++)
//     {        
//         Eigen::VectorXd input_i_base_frame = input_trajectory_base_frame[i];
//         Eigen::VectorXd input_i_world_frame = Eigen::VectorXd::Zero(input_i_base_frame.size());

//         // RCLCPP_INFO_STREAM(node_->get_logger(), "      input_i_base_frame: ");
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          foot contact forces: " << input_i_base_frame.segment(0, 12).transpose());
//         // RCLCPP_INFO_STREAM(node_->get_logger(), "          foot velocities wrt base: " << input_i_base_frame.segment(12, 12).transpose());

//         for (int j = 0; j < 24; j += 3)
//         {
//             input_i_world_frame.segment(j, 3) = transformHelperVector3Stamped(input_i_base_frame.segment(j, 3),
//                                                                                 baseFrameToWorldFrameTransform);            
//         }

//         input_trajectory_world_frame[i] = input_i_world_frame;
//     }

//     // transform terrain normals
//     terrain_normals_trajectory_world_frame.resize(terrain_normals_trajectory_base_frame.size());
//     for (int i = 0; i < terrain_normals_trajectory_base_frame.size(); i++)
//     {
//         Eigen::VectorXd terrainNormals_i_base_frame = terrain_normals_trajectory_base_frame[i];
//         Eigen::VectorXd terrainNormals_i_world_frame = Eigen::VectorXd::Zero(terrainNormals_i_base_frame.size());

//         for (int j = 0; j < 12; j += 3)
//         {
//             terrainNormals_i_world_frame.segment(j, 3) = transformHelperVector3Stamped(terrainNormals_i_base_frame.segment(j, 3),
//                                                                                         baseFrameToWorldFrameTransform);
//         }

//         terrain_normals_trajectory_world_frame[i] = terrainNormals_i_world_frame;
//     }

//     // copy timeTrajectory
//     time_trajectory_world_frame = time_trajectory_base_frame;
// }

void GraphTraverser::clearExperienceBuffers()
{
    for (ExperienceStep * stepPtr : experienceWalk)
    {
        delete stepPtr;
    }

    experienceWalk.clear();
}

void GraphTraverser::clearTrajectories()
{
    state_trajectory_base_frame.clear();
    input_trajectory_base_frame.clear();
    time_trajectory_base_frame.clear();

    state_trajectory_world_frame.clear();
    input_trajectory_world_frame.clear();
    time_trajectory_world_frame.clear();

    terrain_normals_trajectory_base_frame.clear();
    terrain_normals_trajectory_world_frame.clear();

    mode_schedules.clear();
    switching_times.clear();

    failed_step_state_trajectory_base_frame.clear();
    failed_step_input_trajectory_base_frame.clear();
    failed_step_time_trajectory_base_frame.clear();  

    sparseTargetTrajectories_.timeTrajectory.clear();
    sparseTargetTrajectories_.stateTrajectory.clear();
    sparseTargetTrajectories_.inputTrajectory.clear();
    sparseGaitSequence_.clear();

    refinedTargetTrajectories_.timeTrajectory.clear();
    refinedTargetTrajectories_.stateTrajectory.clear();
    refinedTargetTrajectories_.inputTrajectory.clear();

    refinedGaitSequence_.clear();
}

}  // namespace mmp


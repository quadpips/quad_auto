#include <mmp_quadruped/planning/ReferenceTrajectoryPublisher.h>


namespace mmp {

ReferenceTrajectoryPublisher::ReferenceTrajectoryPublisher(const rclcpp::Node::SharedPtr& node,
                                                            const std::string & mpc_topic_prefix,
                                                            const std::string & hyperparametersFile,
                                                            const std::string & gait)
{
    node_ = node;
    gait_ = gait;

    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(hyperparametersFile, pt);
    const std::string trials_prefix = "tracking.";
    ocs2::loadData::loadPtreeValue(pt, params_.T_ahead, trials_prefix + "t_ahead", verbose);
    ocs2::loadData::loadPtreeValue(pt, params_.include_inputs, trials_prefix + "include_inputs", verbose);
    ocs2::loadData::loadPtreeValue(pt, params_.refine_to, trials_prefix + "refine_to", verbose);
    ocs2::loadData::loadPtreeValue(pt, params_.full_ref_traj, trials_prefix + "full_ref_traj", verbose);
    ocs2::loadData::loadPtreeValue(pt, params_.near_goal_dist_threshold, trials_prefix + "near_goal_dist_threshold", verbose);

    if (params_.T_ahead <= 0.0)
    {
        throw std::runtime_error("[ReferenceTrajectoryPublisher::ReferenceTrajectoryPublisher] Error: T_ahead must be positive!");
    }

    gaitSequencePublisher_ =
      node_->create_publisher<ocs2_switched_model_msgs::msg::ScheduledGaitSequence>(mpc_topic_prefix + "_mpc_gait_schedule", 1); // , true

    target_trajectories_publisher_ =
      node_->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(mpc_topic_prefix + "_mpc_target", 1); // ,

    // observation subscriber
    auto observation_callback =
        [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg)
    {
        // ROS_INFO_STREAM("[ReferenceTrajectoryPublisher]: Observation received.");
        // std::lock_guard<std::mutex> lock(latest_observation_mutex_);
        latest_observation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "latest observation is: ");
        // RCLCPP_INFO_STREAM(node_->get_logger(), "      state: " << latest_observation_.state.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "      input: " << latest_observation_.input.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "      time: " << latest_observation_.time);
    };
    observation_sub_ =
        node_->create_subscription<ocs2_msgs::msg::MpcObservation>(mpc_topic_prefix 
        + "_mpc_observation", 1, observation_callback);
}

std::pair<ocs2::TargetTrajectories, switched_model::GaitSchedule::GaitSequence> ReferenceTrajectoryPublisher::prepareReferenceTrajectory(const vector_array_t & state_trajectory, 
                                                                    const vector_array_t & input_trajectory, 
                                                                    const scalar_array_t & time_trajectory, 
                                                                    const std::vector<std::string> & mode_schedules,
                                                                    const std::vector<std::string> & switching_times)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[prepareReferenceTrajectory()]");

    scalar_t running_time_horizon = 0.0;

    ocs2::TargetTrajectories ref_trajectory;

    RCLCPP_INFO_STREAM(node_->get_logger(), "reference trajectory: ");
    for (int i = 0; i < time_trajectory.size(); i++)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "i: " << i);

        scalar_t t_orig = time_trajectory[i]; //.asDouble();
        // RCLCPP_INFO_STREAM(node_->get_logger(), "  t_orig: " << t_orig);
        if (t_orig == 0.0 && i > 0)
        {
            running_time_horizon += time_trajectory[i - 1]; // .asDouble();
            continue; // to avoid double counting end/beginning of subproblems
        }
        scalar_t t_aug = t_orig + running_time_horizon;
        RCLCPP_INFO_STREAM(node_->get_logger(), "      t_aug: " << t_aug);

        ref_trajectory.timeTrajectory.push_back(t_aug);

        comkino_state_t state(state_trajectory[i].size());
        for (int j = 0; j < state_trajectory[i].size(); j++)
            state[j] = state_trajectory[i][j]; // .asDouble();      
        RCLCPP_INFO_STREAM(node_->get_logger(), "      state: " << state.transpose());
        ref_trajectory.stateTrajectory.push_back(state);

        comkino_input_t input(input_trajectory[i].size());
        for (int j = 0; j < input_trajectory[i].size(); j++)
            input[j] = input_trajectory[i][j]; // .asDouble();
        RCLCPP_INFO_STREAM(node_->get_logger(), "      input: " << input.transpose());
        
        if (params_.include_inputs)
        {

        } else
        {
            // input.head(12) = vector_t::Zero(12); // zero out contact forces
            input = vector_t::Zero(input.size());
        }
        
        ref_trajectory.inputTrajectory.push_back(input);
    }

    // prepare mode schedule
    running_time_horizon = 0.0;

    // Gait
    GaitSchedule::GaitSequence gaitSequence;
    using MN = switched_model::ModeNumber;
    switched_model::Gait stance;
    stance.duration = 1.0;
    stance.eventPhases = {};
    stance.modeSequence = {MN::STANCE};

    // std::string gait = "standing_trot";

    if (gait_ == "standing_trot")
    {
        switched_model::Gait partial_trot;
        partial_trot.duration = 1.0; // keeping this long, otherwise trajectory ends early
        partial_trot.eventPhases = {0.05, 0.45};
        partial_trot.modeSequence = {MN::STANCE, MN::LF_RH, MN::STANCE};

        switched_model::Gait standing_trot;
        standing_trot.duration = 1.0;
        standing_trot.eventPhases = {0.05, 0.45, 0.55, 0.95};
        standing_trot.modeSequence = {MN::STANCE, MN::LF_RH, MN::STANCE, MN::RF_LH, MN::STANCE};

        for (int i = 1; i < mode_schedules.size(); i++)
        {
            if (switched_model::string2ModeNumber(mode_schedules[i]) == MN::STANCE)
            {
                if (switched_model::string2ModeNumber(mode_schedules[i - 1]) == MN::RF_LH) // gone through entire cycle
                {
                    gaitSequence.push_back(standing_trot);
                } else if (switched_model::string2ModeNumber(mode_schedules[i - 1]) == MN::LF_RH && // gone through partial cycle and at end
                            i == mode_schedules.size() - 1)
                {
                    gaitSequence.push_back(partial_trot);
                }
            }
        }
    } else if (gait_ == "standing_walk")
    {
        switched_model::Gait first_step;
        first_step.duration = 1.0; // should not set to value greater than 1, this is used as a scaling factor in OCS2, unclear what it represents
        first_step.eventPhases = {0.05, 0.45};
        // first_step.eventPhases = {0.10, 0.40};
        first_step.modeSequence = {MN::STANCE, MN::LF_RF_RH, MN::STANCE};

        switched_model::Gait third_step;
        third_step.duration = 1.0; // should not set to value greater than 1, this is used as a scaling factor in OCS2, unclear what it represents
        third_step.eventPhases = {0.05, 0.45};
        third_step.modeSequence = {MN::STANCE, MN::LF_RF_LH, MN::STANCE};        

        switched_model::Gait static_walk_first_half;
        static_walk_first_half.duration = 1.0; // should not set to value greater than 1, this is used as a scaling factor in OCS2, unclear what it represents
        static_walk_first_half.eventPhases = {0.05, 0.45, 0.55, 0.95};
        static_walk_first_half.modeSequence = {MN::STANCE, MN::LF_RF_RH, MN::STANCE, MN::RF_LH_RH, MN::STANCE};                
    
        switched_model::Gait static_walk_second_half;
        static_walk_second_half.duration = 1.0; // should not set to value greater than 1, this is used as a scaling factor in OCS2, unclear what it represents
        static_walk_second_half.eventPhases = {0.05, 0.45, 0.55, 0.95};
        static_walk_second_half.modeSequence = {MN::STANCE, MN::LF_RF_LH, MN::STANCE, MN::LF_LH_RH, MN::STANCE};                

        RCLCPP_INFO_STREAM(node_->get_logger(), "mode_schedules: ");
        for (int i = 1; i < mode_schedules.size(); i++)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "    " << mode_schedules[i]);

            if (switched_model::string2ModeNumber(mode_schedules[i]) == MN::STANCE)
            {
                if (switched_model::string2ModeNumber(mode_schedules[i - 1]) == MN::LF_LH_RH) // gone through entire cycle
                {
                    RCLCPP_INFO_STREAM(node_->get_logger(), "    adding full four steps");
                    gaitSequence.push_back(static_walk_first_half);
                    gaitSequence.push_back(static_walk_second_half);
                } else if (switched_model::string2ModeNumber(mode_schedules[i - 1]) == MN::LF_RF_LH && // gone through partial cycle and at end
                            i == mode_schedules.size() - 1)
                {
                    RCLCPP_INFO_STREAM(node_->get_logger(), "    adding three steps");
                    gaitSequence.push_back(static_walk_first_half);
                    gaitSequence.push_back(third_step);
                } else if (switched_model::string2ModeNumber(mode_schedules[i - 1]) == MN::RF_LH_RH &&
                            i == mode_schedules.size() - 1)
                {
                    RCLCPP_INFO_STREAM(node_->get_logger(), "    adding two steps");
                    gaitSequence.push_back(static_walk_first_half);
                } else if (switched_model::string2ModeNumber(mode_schedules[i - 1]) == MN::LF_RF_RH &&
                            i == mode_schedules.size() - 1)
                {
                    RCLCPP_INFO_STREAM(node_->get_logger(), "    adding one step");
                    gaitSequence.push_back(first_step);
                }
            }
        }
    }

    for (size_t i = 0; i < ref_trajectory.timeTrajectory.size(); i++)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "    Orig Ref Traj " << i);
        RCLCPP_INFO_STREAM(node_->get_logger(), "       time:" << ref_trajectory.timeTrajectory[i]);
        RCLCPP_INFO_STREAM(node_->get_logger(), "       state:");
        RCLCPP_INFO_STREAM(node_->get_logger(), "           torso orientation: " << ref_trajectory.stateTrajectory[i].segment(0, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           torso position:    " << ref_trajectory.stateTrajectory[i].segment(3, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           torso ang vel: " << ref_trajectory.stateTrajectory[i].segment(6, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           torso lin vel:    " << ref_trajectory.stateTrajectory[i].segment(9, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           LF joints:       " << ref_trajectory.stateTrajectory[i].segment(12, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           RF joints:       " << ref_trajectory.stateTrajectory[i].segment(15, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           LH joints:       " << ref_trajectory.stateTrajectory[i].segment(18, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           RH joints:       " << ref_trajectory.stateTrajectory[i].segment(21, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "       input:");
        RCLCPP_INFO_STREAM(node_->get_logger(), "           LF contact forces: " << ref_trajectory.inputTrajectory[i].segment(0, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           RF contact forces: " << ref_trajectory.inputTrajectory[i].segment(3, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           LH contact forces: " << ref_trajectory.inputTrajectory[i].segment(6, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           RH contact forces: " << ref_trajectory.inputTrajectory[i].segment(9, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           LF foot vel wrt base: " << ref_trajectory.inputTrajectory[i].segment(12, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           RF foot vel wrt base: " << ref_trajectory.inputTrajectory[i].segment(15, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           LH foot vel wrt base: " << ref_trajectory.inputTrajectory[i].segment(18, 3).transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "           RH foot vel wrt base: " << ref_trajectory.inputTrajectory[i].segment(21, 3).transpose());
    }

    for (size_t i = 0; i < gaitSequence.size(); i++)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "    Gait " << i);
        RCLCPP_INFO_STREAM(node_->get_logger(), "       duration: " << gaitSequence[i].duration);
        RCLCPP_INFO_STREAM(node_->get_logger(), "       modeSequence: ");
        for (size_t j = 0; j < gaitSequence[i].modeSequence.size(); j++)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "           mode " << j << ": " << switched_model::modeNumber2String(gaitSequence[i].modeSequence[j]));
        }
        RCLCPP_INFO_STREAM(node_->get_logger(), "       eventPhases: ");
        for (size_t j = 0; j < gaitSequence[i].eventPhases.size(); j++)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "           " << gaitSequence[i].eventPhases[j]);
        }
    }

    // make last stance a bit longer to make sure we finish the swing
    // gaitSequence.at(gaitSequence.size() - 1).duration += 0.5;

    // Make final stance phase longer to allow time to finish swing?
    gaitSequence.push_back(stance);

    return std::make_pair(ref_trajectory, gaitSequence);
}

void ReferenceTrajectoryPublisher::sendReferenceTrajectory(const ocs2::TargetTrajectories & inc_ref_trajectory,
                                                            const switched_model::GaitSchedule::GaitSequence & gaitSequence)
{
    ocs2::TargetTrajectories ref_trajectory = inc_ref_trajectory;

    // set target trajectories to be T_ahead in the future
    // THIS IS A NECESSARY STEP
    RCLCPP_INFO_STREAM(node_->get_logger(), "[ReferenceTrajectoryPublisher::sendReferenceTrajectory] Shifting reference trajectory by T_ahead = " << params_.T_ahead << " seconds.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "    latest_observation_.time: " << latest_observation_.time);

    const scalar_t startTime = (latest_observation_.time + params_.T_ahead); 

    for (auto & time : ref_trajectory.timeTrajectory)
        time += startTime; 


    // RCLCPP_INFO_STREAM(node_->get_logger(), "[ReferenceTrajectoryPublisher::sendReferenceTrajectory] Sending reference trajectory.");
    
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    Ref Traj: ");
    // for (size_t i = 0; i < ref_trajectory.timeTrajectory.size(); i++)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       time = " << ref_trajectory.timeTrajectory[i]
    //                                          << "           state = " << ref_trajectory.stateTrajectory[i].transpose()
    //                                          << "           input = " << ref_trajectory.inputTrajectory[i].transpose());
    // }
    
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    Gait Sequence: ");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "       startTime: " << startTime);
    // for (size_t i = 0; i < gaitSequence.size(); i++)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "           duration = " << gaitSequence[i].duration);
    //     for (size_t j = 0; j < gaitSequence[i].modeSequence.size(); j++)
    //     {
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "           mode " << j << ": " << switched_model::modeNumber2String(gaitSequence[i].modeSequence[j]));
    //     }
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "           eventPhases: ");
    //     for (size_t j = 0; j < gaitSequence[i].eventPhases.size(); j++)
    //     {
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "               " << gaitSequence[i].eventPhases[j]);
    //     }
    // }

    const auto gaitMessage = switched_model::ros_msg_conversions::toMessage(startTime, gaitSequence);

    const auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(ref_trajectory);

    // modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(ref_mode_sequence_template));

    // TARGET TRAJECTORIES MUST BE SENT FIRST, PRETTY TIME SENSITIVE
    target_trajectories_publisher_->publish(mpcTargetTrajectoriesMsg);

    gaitSequencePublisher_->publish(gaitMessage);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "full switchingTimes: ");
    // for (int i = 0; i < ref_mode_sequence_template.switchingTimes.size(); i++)
    //     RCLCPP_INFO_STREAM(node_->get_logger(), i << ": " << ref_mode_sequence_template.switchingTimes[i]);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "full modeSequence: ");
    // for (int i = 0; i < ref_mode_sequence_template.modeSequence.size(); i++)
    //     RCLCPP_INFO_STREAM(node_->get_logger(), i << ": " << ref_mode_sequence_template.modeSequence[i]);

    finalState_ = ref_trajectory.stateTrajectory.back();    
}

bool ReferenceTrajectoryPublisher::isTrackingDoneStateBased()
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[ReferenceTrajectoryPublisher::isTrackingDoneStateBased]");

    comkino_state_t latest_state(latest_observation_.state);
    comkino_input_t latest_input = latest_observation_.input;

    base_coordinate_t torso_pose = getBasePose(latest_state); // .head(6);
    base_coordinate_t final_torso_pose = getBasePose(finalState_); // .head(6);

    vector3_t torso_position = extractTorsoPosition(torso_pose);
    vector3_t final_torso_position = extractTorsoPosition(final_torso_pose);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "torso_position:       " << torso_position.transpose());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "final_torso_position: " << final_torso_position.transpose());

    bool nearGoal = (torso_position - final_torso_position).norm() < params_.near_goal_dist_threshold;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "nearGoal: " << nearGoal);

    // stableContactForces_ = (vector_t(12) << 0.0, 0.0, 40.0, 
    //                                         0.0, 0.0, 40.0, 
    //                                         0.0, 0.0, 40.0, 
    //                                         0.0, 0.0, 40.0).finished();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "stableContactForce_: " << stableContactForce_.transpose());

    double contactForceThreshold = 0.0;
    double jointVelocityThreshold = 0.5;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "latest_input: ");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      FL: " << latest_input.segment(0, 3).transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      FR: " << latest_input.segment(3, 3).transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      RL: " << latest_input.segment(6, 3).transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      RR: " << latest_input.segment(9, 3).transpose());

    bool stable = (latest_input.segment(0, 3).norm() > contactForceThreshold && 
                   latest_input.segment(3, 3).norm() > contactForceThreshold && 
                   latest_input.segment(6, 3).norm() > contactForceThreshold && 
                   latest_input.segment(9, 3).norm() > contactForceThreshold);

                //    latest_input.segment(12, 3).norm() < jointVelocityThreshold && 
                //    latest_input.segment(15, 3).norm() < jointVelocityThreshold && 
                //    latest_input.segment(18, 3).norm() < jointVelocityThreshold && 
                //    latest_input.segment(21, 3).norm() < jointVelocityThreshold);
    // bool stable = ((latest_input.segment(0, 3) - stableContactForces_.segment(0, 3)).norm() < contactForceThreshold && 
    //                (latest_input.segment(3, 3) - stableContactForces_.segment(3, 3)).norm() < contactForceThreshold && 
    //                (latest_input.segment(6, 3) - stableContactForces_.segment(6, 3)).norm() < contactForceThreshold && 
    //                (latest_input.segment(9, 3) - stableContactForces_.segment(9, 3)).norm() < contactForceThreshold && 
    //                latest_input.segment(12, 3).norm() < jointVelocityThreshold && 
    //                latest_input.segment(15, 3).norm() < jointVelocityThreshold && 
    //                latest_input.segment(18, 3).norm() < jointVelocityThreshold && 
    //                latest_input.segment(21, 3).norm() < jointVelocityThreshold);
    
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      nearGoal: " << nearGoal << ", stable: " << stable);
    
    if (nearGoal)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "      Reached near goal position.");
    }

    if (stable)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "      Contact forces or foot velocities are stable.");
    }

    return (nearGoal && stable);
}

}  // namespace mmp

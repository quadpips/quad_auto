/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include <go2_system/state_machine/FSM_State_PerceptiveLocomotionOCS2.h>

#include <ocs2_go2_mpc/Go2Interface.h>

namespace legged_software {
namespace go2_system {

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
  FSM_State_PerceptiveLocomotionOCS2::FSM_State_PerceptiveLocomotionOCS2(const rclcpp::Node::SharedPtr & node, ControlFSMData* _controlFSMData)
  : FSM_State(_controlFSMData, FSM_StateName::PERCEPTIVELOCOMOTIONOCS2, "PERCEPTIVELOCOMOTIONOCS2")
{
  // Instantiate the OCS2 interface
  node_ = node;

  RCLCPP_INFO_STREAM(node->get_logger(), "[FSM_State_PerceptiveLocomotionOCS2()] started ");

  // initTime_ = node_->get_clock()->now().seconds();

  const std::string taskFile = node_->get_parameter("taskFile").as_string();
  const std::string urdfFile = node_->get_parameter("urdfFile").as_string();
  const std::string frameFile = node_->get_parameter("frameFile").as_string();
  const std::string sqpFile = node_->get_parameter("sqpFile").as_string();
  const std::string controllerConfigFile = node_->get_parameter("controllerConfigFile").as_string();
  
  std::string urdfString = go2::getUrdfString(urdfFile);

  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM_State_PerceptiveLocomotionOCS2()] urdfFile: " << urdfFile);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM_State_PerceptiveLocomotionOCS2()] urdfString: " << urdfString);

  setupLeggedRobotInterface(taskFile, urdfString, frameFile); // , envFile

  setupMpc(taskFile, sqpFile);

  setupMrt(sqpFile);

  // Visualization
  visualizer_ = std::make_shared<CustomQuadrupedVisualizer>(legged_interface_->getKinematicModel(), 
                                                              legged_interface_->getJointNames(), 
                                                              legged_interface_->getBaseName(),
                                                              node_,
                                                              1000,
                                                              false); // turning off tf publishing

  observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);

  // HO based Whole body control
  wbcPtr_ = getWbcUnique(controllerConfigFile, 
                          urdfString, 
                          legged_interface_->getComModel(), 
                          legged_interface_->getKinematicModel(), 
                          legged_interface_->getJointNames());

  this->turnOnAllSafetyChecks();

  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM_State_PerceptiveLocomotionOCS2()] finished ");

  pastObservation_.time = 0.0;
  pastObservation_.mode = switched_model::ModeNumber::STANCE;
  pastObservation_.state = vector_t::Zero(switched_model::STATE_DIM);
  pastObservation_.input = vector_t::Zero(switched_model::INPUT_DIM);
}

FSM_State_PerceptiveLocomotionOCS2::~FSM_State_PerceptiveLocomotionOCS2()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "[~FSM_State_PerceptiveLocomotionOCS2()]");

  controller_running_ = false;
  if (ocs2_thread_.joinable())
    ocs2_thread_.join();
}

void FSM_State_PerceptiveLocomotionOCS2::onEnter() 
{
  // std::cout << "pre-onEnter cout" << std::endl;
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] onEnter started \n");
  // std::cout << "post-onEnter cout" << std::endl;

  this->perceptiveOCS2Safe = true;

  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // auto referenceManager = legged_interface_->getSwitchedModelModeScheduleManagerPtr();

  // Initial state
  SystemObservation initial_observation = generateSystemObservation();
  initial_observation.mode = switched_model::ModeNumber::STANCE; // Set the initial mode to stance

  // initial_observation.time = 0.0;
  // initial_observation.mode = switched_model::ModeNumber::STANCE;
  // initial_observation.state = extractCentroidalState(getRbdState()); // legged_interface_->getInitialState(); // 
  // initial_observation.input.setZero(switched_model::INPUT_DIM); // 

  RCLCPP_INFO_STREAM(node_->get_logger(), "initial_observation: ");
  printObservation(initial_observation);

  TargetTrajectories target_trajectories( { initial_observation.time }, 
                                          { initial_observation.state },
                                          { initial_observation.input });

  RCLCPP_INFO_STREAM(node_->get_logger(), "Setting up the controller ...");

  // Set the first observation and command and wait for optimization to finish
  mpc_mrt_interface_->reset();
  mpc_mrt_interface_->resetMpcNode(target_trajectories);

  mpc_mrt_interface_->setCurrentObservation(initial_observation);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for the initial policy ...");
  while (!mpc_mrt_interface_->initialPolicyReceived() && rclcpp::ok())
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Advancing MPC ...");
    mpc_mrt_interface_->advanceMpc();
    RCLCPP_INFO_STREAM(node_->get_logger(), "Sleeping ...");
    rclcpp::WallRate(mpcPtr_->settings().mrtDesiredFrequency_).sleep();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Initial policy has been received.");

  mpc_running_ = true;

  // Reset iteration counter
  iter = 0;

  pastObservation_ = initial_observation;

  RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] onEnter finished \n");

  return;
}

void FSM_State_PerceptiveLocomotionOCS2::setupLeggedRobotInterface(const std::string& taskFile, 
                                                                      const std::string& urdfString,
                                                                      const std::string& frameFile)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] setupLeggedRobotInterface");

  legged_interface_ = go2::getGo2Interface(urdfString, taskFile, frameFile);

  return;
}

void FSM_State_PerceptiveLocomotionOCS2::setupMpc(const std::string& taskFile, 
                                                     const std::string& sqpFile)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] setupMpc");

  if (!legged_interface_)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"[FSM PERCEPTIVE LOCOMOTION OCS2] setupMpc: legged_interface_ is not initialized");
    return;
  }

  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile);
  ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile);
  mpcPtr_ = std::make_shared<ocs2::SqpMpc>(mpcSettings, 
                                            sqpSettings,
                                            legged_interface_->getOptimalControlProblem(),
                                            legged_interface_->getInitializer());

  auto solverModules = legged_interface_->getSynchronizedModules();

  // Gait Receiver
  auto gaitReceiver =
      std::make_shared<GaitReceiver>(node_, 
                                      legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), 
                                      robotName);
  solverModules.push_back(gaitReceiver);

  // Terrain Receiver
  RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] Adding TerrainReceiver");
  auto terrainReceiver = std::make_shared<CustomTerrainReceiverSynchronizedModule>(
      legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), node_);
  solverModules.push_back(terrainReceiver);

  // Terrain plane visualization
  auto terrainVisualizer = std::make_shared<CustomTerrainPlaneVisualizerSynchronizedModule>(
      legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node_);
  solverModules.push_back(terrainVisualizer);

  // Swing planner
  auto swingPlanningVisualizer = std::make_shared<CustomSwingPlanningVisualizer>(
      legged_interface_->getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node_);
  solverModules.push_back(swingPlanningVisualizer);

  // ROS ReferenceManager; The ReferenceManager will subscribe to "topicPrefix_mode_schedule" and "topicPrefix_mpc_target"
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, 
                                                                            legged_interface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node_);
  mpcPtr_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // MPC
  mpcPtr_->getSolverPtr()->setSynchronizedModules(solverModules);

  return;
}

void FSM_State_PerceptiveLocomotionOCS2::setupMrt(const std::string& sqpFile)
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] setupMrt");

  if (!legged_interface_)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"[FSM PERCEPTIVE LOCOMOTION OCS2] setupMrt: legged_interface_ is not initialized");
    return;
  }

  if (!mpcPtr_)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"[FSM PERCEPTIVE LOCOMOTION OCS2] setupMrt: mpcPtr_ is not initialized");
    return;
  }

  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpcPtr_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

  // int64_t mpc_freq_throttle_msec = 100;
  controller_running_ = true;

  ocs2_thread_ = std::thread([&]() 
  {
    float min_mpc_rate = 0.5 * mpcPtr_->settings().mpcDesiredFrequency_; // 1/2 of desired frequency
    float max_mpc_time = 1.0 / min_mpc_rate;

    std::chrono::steady_clock::time_point pre_mpc_time, post_mpc_time, post_sleep_time;
    int64_t mpc_time_microsec, loop_time_microsec;
    double mpc_time_sec, loop_time_sec;

    while (controller_running_)
    {
      // RCLCPP_INFO_STREAM(node_->get_logger(), "[Ocs2 MPC thread] Running \n");

      try
      {
        // Run as fast as possible
        if (mpc_running_)
        {
          // RCLCPP_INFO_STREAM(node_->get_logger(), "[Ocs2 MPC thread] Running MPC \n");

          pre_mpc_time = std::chrono::steady_clock::now();
          mpc_mrt_interface_->advanceMpc();
          post_mpc_time = std::chrono::steady_clock::now();

          mpc_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(post_mpc_time - pre_mpc_time).count();
          mpc_time_sec = 1.0e-6 * mpc_time_microsec;
          // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(),
          //                             *node_->get_clock(),
          //                             mpc_freq_throttle_msec,
          //                             "mpc freq is: " << (1.0 / mpc_time_sec) << " Hz");

          // rclcpp::WallRate(mpcPtr_->settings().mpcDesiredFrequency_).sleep();

          // post_sleep_time = std::chrono::steady_clock::now();
          // loop_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(post_sleep_time - pre_mpc_time).count();
          // loop_time_sec = 1.0e-6 * loop_time_microsec;
          
          // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(),
          //                             *node_->get_clock(),
          //                             1000,
          //                             "[Ocs2 MPC thread] MPC time (Hz): " << (1.0 / mpc_time_sec) << ", loop time (Hz): " << (1.0 / loop_time_sec));
          // if (loop_time_sec > (2.0 / mpcPtr_->settings().mpcDesiredFrequency_))
          // {
          //   RCLCPP_WARN_STREAM(node_->get_logger(), "[Ocs2 MPC thread] Loop time is larger than 2x desired frequency.");
          // }

          // if (mpc_time_sec > max_mpc_time)
          // {
          //   RCLCPP_WARN_STREAM(node_->get_logger(), "[Ocs2 MPC thread] Loop time is very large (" << mpc_time_sec << " > " << max_mpc_time << "), deeming unsafe and stopping controller.");
          //   this->perceptiveOCS2Safe = false;
          //   mpc_running_ = false;
          //   controller_running_ = false;
          //   sendZeroCommand();
          // }
        }
      }
      catch (const std::exception& e)
      {
        controller_running_ = false;
        RCLCPP_ERROR_STREAM(node_->get_logger(),"[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });

  // ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile);
  // setThreadPriority(sqpSettings.threadPriority, ocs2_thread_);

  return;
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_PerceptiveLocomotionOCS2::run() 
{
  if (!this->perceptiveOCS2Safe || !mpc_running_ || !controller_running_)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run(): Perceptive Locomotion OCS2 is not safe! Will not run.");
    sendZeroCommand();
    return;
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run() started \n");

  // tNow_ = node_->get_clock()->now().seconds() - initTime_;
  // current_observation_.time += this->_data->userParameters->controller_dt;
  // RCLCPP_INFO_STREAM(node_->get_logger(), "tNow_: " << tNow_);

  // timeSinceLastLog_ += this->_data->userParameters->controller_dt;
  // bool logCheck = (timeSinceLastLog_ > 0.01);
  SystemObservation observation = generateSystemObservation();
  observation.time += this->_data->userParameters->controller_dt; // controller_dt dynamically updated in bridge
  observation.mode = desiredMode; // Set the mode to the desired mode

  // wrap RPY (not really sure if we need)
  scalar_t roll_curr = observation.state(0);
  scalar_t pitch_curr = observation.state(1);
  scalar_t yaw_curr = observation.state(2);
  scalar_t roll_last = pastObservation_.state(0);
  scalar_t pitch_last = pastObservation_.state(1);
  scalar_t yaw_last = pastObservation_.state(2);
  observation.state(0) = roll_last + angles::shortest_angular_distance(roll_last, roll_curr);
  observation.state(1) = pitch_last + angles::shortest_angular_distance(pitch_last, pitch_curr);
  observation.state(2) = yaw_last + angles::shortest_angular_distance(yaw_last, yaw_curr);

  //////////////////////////
  // pre-MPC safety check //
  //////////////////////////

  RCLCPP_INFO_STREAM(node_->get_logger(), "   observation:");
  printObservation(observation);  

  // state
  // if (!stateSafetyCheck(observation.state))
  // {
  //   RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run(): Pre-MPC state safety check failed.");
  //   sendZeroCommand();
  //   this->perceptiveOCS2Safe = false;
  //   mpc_running_ = false;
  //   controller_running_ = false;
  //   return;
  // }

  // // input
  // if (!inputSafetyCheck(observation.input))
  // {
  //   RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run(): Pre-MPC input safety check failed.");
  //   sendZeroCommand();
  //   this->perceptiveOCS2Safe = false;
  //   mpc_running_ = false;
  //   controller_running_ = false;    
  //   return;
  // }

  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(observation);

  // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // if (logCheck)
  // {
  //   RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run()");

  //   // RCLCPP_INFO_STREAM(node_->get_logger(), "   measured_rbd_state: ");
  //   // printState(measured_rbd_state);

  // }

  // Evaluate the current policy
  vector_t desiredState;
  mpc_mrt_interface_->evaluatePolicy(observation.time, observation.state, desiredState, desiredInput, desiredMode);
  // planned_mode_ = desiredMode;

  // TODO check health of policy

  // Need to do this if we have no active foot sensor and are using EKF for state estimation
  switched_model::contact_flag_t feet_contact_state = modeNumber2StanceLeg(desiredMode);
  Eigen::Vector4d planned_contactState(0,0,0,0);
  for(int leg=0; leg<4; leg++)
  {
    planned_contactState[leg] = feet_contact_state[leg]? 0.5 : 0.0;
  }
  this->_data->_stateEstimator->setContactEstimate(planned_contactState);
  // TODO: analyze PerformanceIndex of MPC

  RCLCPP_INFO_STREAM(node_->get_logger(), "desired:");
  RCLCPP_INFO_STREAM(node_->get_logger(), "   mode:       " << modeNumber2String(desiredMode));

  RCLCPP_INFO_STREAM(node_->get_logger(), "   state: ");
  printState(desiredState);

  RCLCPP_INFO_STREAM(node_->get_logger(), "   input: ");
  printInput(desiredInput);

  //////////////////////////
  // post-MPC safety check //
  //////////////////////////

  // state
  // if (!stateSafetyCheck(desiredState))
  // {
  //   RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run(): Post-MPC state safety check failed.");
  //   sendZeroCommand();
  //   this->perceptiveOCS2Safe = false;
  //   mpc_running_ = false;
  //   controller_running_ = false;    
  //   return;
  // }

  // input
  // if (!inputSafetyCheck(desiredInput))
  // {
  //   RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run(): Post-MPC input safety check failed.");
  //   sendZeroCommand();
  //   this->perceptiveOCS2Safe = false;
  //   mpc_running_ = false;
  //   controller_running_ = false;    
  //   return;
  // }

  // if (logCheck)
  // {
  //   // RCLCPP_INFO_STREAM(node_->get_logger(), "current_observation_ (post-MPC, pre-WBC):");
  //   // printObservation(current_observation_);

  // }

  constexpr scalar_t time_eps = 1e-4;
  vector_t dummyState;
  vector_t dummyInput;
  size_t dummyMode;
  mpc_mrt_interface_->evaluatePolicy(observation.time + time_eps,
                                      observation.state, 
                                      dummyState, 
                                      dummyInput, 
                                      dummyMode);

  vector_t joint_accelerations = (dummyInput.tail<12>() - desiredInput.tail<12>()) / time_eps;

  // if (logCheck)
  // {                 
  //   // RCLCPP_INFO_STREAM(node_->get_logger(), "planned_contactState: " << planned_contactState.transpose());
                     
  RCLCPP_INFO_STREAM(node_->get_logger(), "   dummy input: ");
  printInput(dummyInput);

  // RCLCPP_INFO_STREAM(node_->get_logger(), "joint accelerations:");
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    LF_HAA acceleration: " << joint_accelerations[0]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    LF_HFE acceleration: " << joint_accelerations[1]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    LF_KFE acceleration: " << joint_accelerations[2]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    RF_HAA acceleration: " << joint_accelerations[3]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    RF_HFE acceleration: " << joint_accelerations[4]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    RF_KFE acceleration: " << joint_accelerations[5]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    LH_HAA acceleration: " << joint_accelerations[6]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    LH_HFE acceleration: " << joint_accelerations[7]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    LH_KFE acceleration: " << joint_accelerations[8]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    RH_HAA acceleration: " << joint_accelerations[9]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    RH_HFE acceleration: " << joint_accelerations[10]);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "    RH_KFE acceleration: " << joint_accelerations[11]);
  // }


  // TODO: check desired stuff
  // If desired state is too far, fail
  // If desired input is too large, fail
  // If joint accelerations are too large, fail

  // Updating WBC
  // std::chrono::steady_clock::time_point pre_wbc_time = std::chrono::steady_clock::now();
  int flag = -1; // -1: WBC not run, 0: success, 1: WBC failed, 2: WBC QP init failed
  std::vector<JointCommand> commands = wbcPtr_->getCommandMessage(observation.time, 
                                                                  observation.state, 
                                                                  observation.input, 
                                                                  observation.mode,
                                                                  desiredState, 
                                                                  desiredInput, 
                                                                  desiredMode, 
                                                                  joint_accelerations,
                                                                  flag);

  RCLCPP_INFO_STREAM(node_->get_logger(), "joint commands: ");  
  for (int i = 0; i < commands.size(); i++)
  {
 
    RCLCPP_INFO_STREAM(node_->get_logger(), "joint: " << commands[i].joint_name);
    RCLCPP_INFO_STREAM(node_->get_logger(), "  desired position: " << commands[i].desired_position);
    RCLCPP_INFO_STREAM(node_->get_logger(), "  desired velocity: " << commands[i].desired_velocity);
    RCLCPP_INFO_STREAM(node_->get_logger(), "  kp:               " << commands[i].kp);
    RCLCPP_INFO_STREAM(node_->get_logger(), "  kd:               " << commands[i].kd);
    RCLCPP_INFO_STREAM(node_->get_logger(), "  torque_ff:        " << commands[i].torque_ff);

    // vector_t jointPositions = observation.state.segment(12,12);
    // vector_t jointVelocities = observation.input.segment(12,12);

    // float torque = commands[i].torque_ff + commands[i].kp * (commands[i].desired_position - jointPositions[i]) +
    //                commands[i].kd * (commands[i].desired_velocity - jointVelocities[i]);

    // if (i == 0 || i == 3 || i == 6 || i == 9)
    // {
    //   if (std::abs(torque) > maxTorqueObserved_[0])
    //   {
    //     maxTorqueObserved_[0] = std::abs(torque);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "New max torque observed for HAA joints: " << maxTorqueObserved_[0]);
    //   }
    // } else if (i == 1 || i == 4 || i == 7 || i == 10)
    // {
    //   if (std::abs(torque) > maxTorqueObserved_[1])
    //   {
    //     maxTorqueObserved_[1] = std::abs(torque);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "New max torque observed for HFE joints: " << maxTorqueObserved_[1]);
    //   }
    // } else if (i == 2 || i == 5 || i == 8 || i == 11)
    // {
    //   if (std::abs(torque) > maxTorqueObserved_[2])
    //   {
    //     maxTorqueObserved_[2] = std::abs(torque);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "New max torque observed for KFE joints: " << maxTorqueObserved_[2]);
    //   }
    // }
  }

  // Post-WBC safety check 
  if (flag != 0) 
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "WBC failed with flag: " << flag);
    sendZeroCommand();
    this->perceptiveOCS2Safe = false;
    mpc_running_ = false;
    controller_running_ = false;    
    return;
  }

  // if (!torqueSafetyCheck(commands, observation.state, observation.input)) 
  // {
  //   RCLCPP_WARN_STREAM(node_->get_logger(), "Torque safety check failed!");
  //   sendZeroCommand();
  //   this->perceptiveOCS2Safe = false;
  //   mpc_running_ = false;
  //   controller_running_ = false;    
  //   return;
  // }

  // std::chrono::steady_clock::time_point post_wbc_time = std::chrono::steady_clock::now();

  // int64_t wbc_time = std::chrono::duration_cast<std::chrono::microseconds>(post_wbc_time - pre_wbc_time).count();
  // double wbc_time_sec = wbc_time / 1.0e6; 
  // int wbc_freq_throttle_msec = 5000;

  // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), 
  //                             *node_->get_clock(), 
  //                             wbc_freq_throttle_msec, 
  //                             "wbc freq is: " << (1.0 / wbc_time_sec) << " Hz");

  vector_t torque(12);
  vector_t joint_kps(12);
  vector_t joint_kds(12);
  vector_t pos_des(12);
  vector_t vel_des(12);

  // if (logCheck)
  // {      
  // RCLCPP_INFO_STREAM(node_->get_logger(), "joint commands: ");
  // }
  
  for (int i = 0; i < commands.size(); i++)
  {
    // if (logCheck)
    // {    
    // RCLCPP_INFO_STREAM(node_->get_logger(), "joint: " << commands[i].joint_name);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  desired position: " << commands[i].desired_position);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  desired velocity: " << commands[i].desired_velocity);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  kp:               " << commands[i].kp);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  kd:               " << commands[i].kd);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  torque_ff:        " << commands[i].torque_ff);
    // }

    torque[i] = commands[i].torque_ff;
    joint_kps[i] = commands[i].kp;
    joint_kds[i] = commands[i].kd;
    pos_des[i] = commands[i].desired_position;
    vel_des[i] = commands[i].desired_velocity;
  }

  // Send torques to LegController
  for (int leg = 0; leg < 4; ++leg) 
  {
    for (int jidx = 0; jidx < 3; ++jidx) 
    {
      this->_data->_legController->commands[leg].tauFeedForward[jidx] = torque[3*leg + jidx];
    
      this->_data->_legController->commands[leg].kpJoint(jidx, jidx) = joint_kps[3*leg + jidx];
      
      this->_data->_legController->commands[leg].kdJoint(jidx, jidx) = joint_kds[3*leg + jidx];

      this->_data->_legController->commands[leg].qDes[jidx] = pos_des[3 * leg + jidx];

      this->_data->_legController->commands[leg].qdDes[jidx] = vel_des[3 * leg + jidx];
    }

    // if (logCheck)
    // {        
    // RCLCPP_INFO_STREAM(node_->get_logger(), "leg: " << leg);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  tauFeedForward: " << this->_data->_legController->commands[leg].tauFeedForward.transpose());
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "  forceFeedForward: " << this->_data->_legController->commands[leg].forceFeedForward.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  kpJoint: " << this->_data->_legController->commands[leg].kpJoint);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  kdJoint: " << this->_data->_legController->commands[leg].kdJoint);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  qDes: " << this->_data->_legController->commands[leg].qDes.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  qdDes: " << this->_data->_legController->commands[leg].qdDes.transpose());
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "  kpCartesian: " << this->_data->_legController->commands[leg].kpCartesian);
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "  kdCartesian: " << this->_data->_legController->commands[leg].kdCartesian);
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "  pDes: " << this->_data->_legController->commands[leg].pDes.transpose());
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "  vDes: " << this->_data->_legController->commands[leg].vDes.transpose());
    // }
  }

  // Visualization
  visualizer_->update(observation, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observation_publisher_->publish(ros_msg_conversions::createObservationMsg(observation));

  pastObservation_ = observation;

  // if (logCheck)
  // {     
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] run ended \n");
  //   timeSinceLastLog_ = 0.0;
  // }

  return;
}

SystemObservation FSM_State_PerceptiveLocomotionOCS2::generateSystemObservation()
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] generateSystemObservation \n");

  SystemObservation observation;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Getting time");
  
  observation.time = pastObservation_.time;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Getting state");

  vector_t measured_rbd_state = getRbdState();
  observation.state = extractCentroidalState(measured_rbd_state);

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Getting mode");

  // HOLDING SPACE FOR MODE UPDATE
  observation.mode = switched_model::ModeNumber::STANCE; // Default mode, will be updated later

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Getting input");

  observation.input.setZero(switched_model::INPUT_DIM);
  observation.input.head(12) = desiredInput.head(12); // Set the input to the desired input
  observation.input.tail(12) = measured_rbd_state.tail(12); // joint velocities

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Generated observation: ");

  return observation;
}

vector_t FSM_State_PerceptiveLocomotionOCS2::getRbdState()
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] getRbdState \n");

  ///////////////
  // NEW ORDER //
  ///////////////

  // [0-2]: Base orientation in world frame (roll, pitch, yaw)
  // [3-5]: Base position in world frame (x, y, z)
  // [6-8]: Base angular velocity in base frame (roll, pitch, yaw)
  // [9-11]: Base linear velocity in base frame (x, y, z)
  // [12-23]: Joint positions for LF, RF, LH, RH (12 joints)
  // [24-35]: Joint velocities for LF, RF, LH, RH (12 joints)

  // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *(node_->get_clock()), 200, 
  // "EKF position (world): [" << this->_data->_stateEstimator->getResult().position[0] << ", " << this->_data->_stateEstimator->getResult().position[1] << ", " << this->_data->_stateEstimator->getResult().position[2] << "], \n" <<
  // "    orientation (world to base, wxyz): [" << this->_data->_stateEstimator->getResult().orientation[0] << ", " << this->_data->_stateEstimator->getResult().orientation[1] << ", " << this->_data->_stateEstimator->getResult().orientation[2] << ", " << this->_data->_stateEstimator->getResult().orientation[3] << "], \n" <<
  // "    linear twist (body): [" << this->_data->_stateEstimator->getResult().vBody[0] << ", " << this->_data->_stateEstimator->getResult().vBody[1] << ", " << this->_data->_stateEstimator->getResult().vBody[2] << "], \n" <<
  // "    linear twist (world): [" << this->_data->_stateEstimator->getResult().vWorld[0] << ", " << this->_data->_stateEstimator->getResult().vWorld[1] << ", " << this->_data->_stateEstimator->getResult().vWorld[2] << "], \n" <<
  // "    angular twist (body): [" << this->_data->_stateEstimator->getResult().omegaBody[0] << ", " << this->_data->_stateEstimator->getResult().omegaBody[1] << ", " << this->_data->_stateEstimator->getResult().omegaBody[2] << "], \n" <<
  // "    angular twist (world): [" << this->_data->_stateEstimator->getResult().omegaWorld[0] << ", " << this->_data->_stateEstimator->getResult().omegaWorld[1] << ", " << this->_data->_stateEstimator->getResult().omegaWorld[2] << "], \n" <<
  // "    linear acc (body): [" << this->_data->_stateEstimator->getResult().aBody[0] << ", " << this->_data->_stateEstimator->getResult().aBody[1] << ", " << this->_data->_stateEstimator->getResult().aBody[2] << "], \n" <<
  // "    linear acc (world): [" << this->_data->_stateEstimator->getResult().aWorld[0] << ", " << this->_data->_stateEstimator->getResult().aWorld[1] << ", " << this->_data->_stateEstimator->getResult().aWorld[2] << "] \n");

  Eigen::VectorXd rbd_state(36);
  
  // RCLCPP_INFO_STREAM(node_->get_logger(), "Filling rpy");

  // Base orientation in world frame (roll, pitch, yaw)
  Eigen::Vector3d rpy = this->_data->_stateEstimator->getResult().rpy; // Base orientation in world frame (roll, pitch, yaw)
  rbd_state.segment(0, 3) = rpy; // Base orientation in world frame (roll, pitch, yaw)

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Filling position");

  // Base position in world frame (x, y, z)
  rbd_state.segment(3, 3) = this->_data->_stateEstimator->getResult().position;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Filling angular velocity");

  // Base angular velocity in base frame (roll, pitch, yaw)
  rbd_state.segment(6, 3) = this->_data->_stateEstimator->getResult().omegaBody;
  
  // RCLCPP_INFO_STREAM(node_->get_logger(), "Filling linear velocity");

  // Base linear velocity in base frame (x, y, z)
  rbd_state.segment(9, 3) = this->_data->_stateEstimator->getResult().vBody;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Filling joint positions and velocities");

  for (int i = 0; i < 3; ++i)
  {
    for (int leg = 0; leg < 4; ++leg)
    {
      // joint positions
      rbd_state[12 + 3*leg + i] = ((LegControllerData*)this->_data->_legController->datas[leg])->q[i];
      
      // joint velocities
      rbd_state[24 + 3*leg + i] = ((LegControllerData*)this->_data->_legController->datas[leg])->qd[i];
    }
  }

  return rbd_state.template cast<double>();
}

vector_t FSM_State_PerceptiveLocomotionOCS2::extractCentroidalState(const vector_t & rbdState)
{
  return rbdState.head(24);
}

void FSM_State_PerceptiveLocomotionOCS2::printObservation(const SystemObservation & observation)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "  time: ");
  RCLCPP_INFO_STREAM(node_->get_logger(), "     " << observation.time);
  RCLCPP_INFO_STREAM(node_->get_logger(), "  mode: ");
  RCLCPP_INFO_STREAM(node_->get_logger(), "     " << modeNumber2String(observation.mode));
  RCLCPP_INFO_STREAM(node_->get_logger(), "  state: ");
  printState(observation.state);
  RCLCPP_INFO_STREAM(node_->get_logger(), "  input: ");
  printInput(observation.input);
}

void FSM_State_PerceptiveLocomotionOCS2::printState(const vector_t & state)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "     torso orientation: " << state.segment(0,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     torso position:    " << state.segment(3,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     torso angular vel: " << state.segment(6,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     torso linear vel:  " << state.segment(9,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     LF joint pos:      " << state.segment(12,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     RF joint pos:      " << state.segment(15,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     LH joint pos:      " << state.segment(18,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     RH joint pos:      " << state.segment(21,3).transpose());
}


void FSM_State_PerceptiveLocomotionOCS2::printInput(const vector_t & input)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "     LF contact force:  " << input.segment(0,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     RF contact force:  " << input.segment(3,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     LH contact force:  " << input.segment(6,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     RH contact force:  " << input.segment(9,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     LF joint vel:      " << input.segment(12,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     RF joint vel:      " << input.segment(15,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     LH joint vel:      " << input.segment(18,3).transpose());
  RCLCPP_INFO_STREAM(node_->get_logger(), "     RH joint vel:      " << input.segment(21,3).transpose());
}

void FSM_State_PerceptiveLocomotionOCS2::sendZeroCommand()
{
  vector_t priorJointState = pastObservation_.state.segment(12, 12);
  // vector_t defaultJointState = legged_interface_->getInitialState().segment(12, 12);
  // vector_t passiveJointState(12);
  // passiveJointState << 0.0, 1.40, -2.70, 0.0, 1.40, -2.70, 0.0, 1.40, -2.70, 0.0, 1.40, -2.70; // Lying pose

  // Send torques to LegController
  for (int leg = 0; leg < 4; ++leg) 
  {
    for (int jidx = 0; jidx < 3; ++jidx) 
    {
      this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.0;

      this->_data->_legController->commands[leg].kpJoint(jidx, jidx) = 0.0;

      this->_data->_legController->commands[leg].kdJoint(jidx, jidx) = 0.0;

      this->_data->_legController->commands[leg].qDes[jidx] = priorJointState[3 * leg + jidx];

      this->_data->_legController->commands[leg].qdDes[jidx] = 0.0;
    }
  }
}

bool FSM_State_PerceptiveLocomotionOCS2::stateSafetyCheck(const vector_t & state) 
{
  // Torso pose
  vector_t rpy = state.segment(0,3);
  vector_t position = state.segment(3,3);

  double maxRollPitch = M_PI / 12.0; // 15 degrees
  if (std::isnan(rpy[0]) || std::isnan(rpy[1]) || std::isnan(rpy[2]) ||
      std::isnan(position[0]) || std::isnan(position[1]) || std::isnan(position[2]))
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: NaN in torso pose: rpy " << rpy.transpose() << ", position " << position.transpose());
    return false;
  }
  
  if (std::abs(rpy[0]) > maxRollPitch) // Roll
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: Roll angle too large: " << rpy[0]);
    return false;
  }

  if (std::abs(rpy[1]) > maxRollPitch) // Pitch
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: Pitch angle too large: " << rpy[1]);
    return false;
  }

  double minHeight = 0.15; // meters
  if (position[2] < minHeight) // Height
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: Height too low: " << position[2]);
    return false;
  }

  // Torso velocity
  vector_t torsoAngVel = state.segment(6,3);
  vector_t torsoLinVel = state.segment(9,3);
  if (std::isnan(torsoAngVel[0]) || std::isnan(torsoAngVel[1]) || std::isnan(torsoAngVel[2]) ||
      std::isnan(torsoLinVel[0]) || std::isnan(torsoLinVel[1]) || std::isnan(torsoLinVel[2]))
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: NaN in torso velocity");
    return false;
  }

  double maxTorsoAngVel = 5.0; // rad/s (not sure about reasonable value)
  if (std::abs(torsoAngVel[0]) > maxTorsoAngVel || std::abs(torsoAngVel[1]) > maxTorsoAngVel || std::abs(torsoAngVel[2]) > maxTorsoAngVel)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: Torso angular velocity too high: " << torsoAngVel.transpose());
    return false;
  }

  double maxTorsoLinVel = 1.0; // m/s (not sure about reasonable value)
  if (std::abs(torsoLinVel[0]) > maxTorsoLinVel || std::abs(torsoLinVel[1]) > maxTorsoLinVel || std::abs(torsoLinVel[2]) > maxTorsoLinVel)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: Torso linear velocity too high: " << torsoLinVel.transpose());
    return false;
  }

  // Joint positions
  vector_t jointPositions = state.segment(12,12);

  if (std::isnan(jointPositions[0]) || std::isnan(jointPositions[1]) || std::isnan(jointPositions[2]) ||
      std::isnan(jointPositions[3]) || std::isnan(jointPositions[4]) || std::isnan(jointPositions[5]) ||
      std::isnan(jointPositions[6]) || std::isnan(jointPositions[7]) || std::isnan(jointPositions[8]) ||
      std::isnan(jointPositions[9]) || std::isnan(jointPositions[10]) || std::isnan(jointPositions[11]))
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: NaN in joint positions: " << jointPositions.transpose());
    return false;
  }

  vector_t jointPositionLowerLimits = legged_interface_->modelSettings().lowerJointLimits_;
  vector_t jointPositionUpperLimits = legged_interface_->modelSettings().upperJointLimits_;

  for (int i = 0; i < 12; i++)
  {
    if (jointPositions[i] < jointPositionLowerLimits[i] || jointPositions[i] > jointPositionUpperLimits[i])
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] stateSafetyCheck: Joint position out of limits for joint " << i 
                              << ": position " << jointPositions[i] 
                              << ", limits [" << jointPositionLowerLimits[i] << ", " << jointPositionUpperLimits[i] << "]");
      return false;
    }
  }

  return true;
}

bool FSM_State_PerceptiveLocomotionOCS2::inputSafetyCheck(const vector_t & input) 
{
  // Joint velocities
  vector_t jointVelocities = input.segment(12,12);

  vector_t jointVelocityLimits = legged_interface_->modelSettings().jointVelocityLimits;

  for (int i = 0; i < 12; i++)
  {
    if (std::isnan(jointVelocities[i]))
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] inputSafetyCheck: NaN in joint velocities: " << jointVelocities.transpose());
      return false;
    }

    if (std::abs(jointVelocities[i]) > jointVelocityLimits[i])
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] inputSafetyCheck: Joint velocity too high for joint " << i 
                              << ": velocity " << jointVelocities[i] 
                              << ", limit " << jointVelocityLimits[i]);
      return false;
    }
  }

  // Contact forces
  vector_t contactForces = input.segment(0,12);
  double maxContactForce = 350.0; // N, not sure about reasonable value

  for (int i = 0; i < 12; i++)
  {
    if (std::isnan(contactForces[i]))
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] inputSafetyCheck: NaN in contact forces: " << contactForces.transpose());
      return false;
    }

    if (std::abs(contactForces[i]) > maxContactForce)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] inputSafetyCheck: Contact force too high for index " << i 
                              << ": force " << contactForces[i] 
                              << ", limit " << maxContactForce);
      return false;
    }
  }

  return true;
}

bool FSM_State_PerceptiveLocomotionOCS2::torqueSafetyCheck(const std::vector<JointCommand> & commands,
                                                              const vector_t & state, const vector_t & input) 
{
  vector_t jointPositions = state.segment(12,12);
  vector_t jointVelocities = input.segment(12,12);
  vector_t jointTorqueLimits = legged_interface_->modelSettings().jointTorqueLimits;

  for (int i = 0; i < commands.size(); i++)
  {
    double tolerance = 1.05; // 5% tolerance

    // Use only feedforward torque for safety check
    double torque_ff = commands[i].torque_ff;

    if (std::isnan(torque_ff))
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] torqueSafetyCheck: NaN in torque feedforward command for joint " << commands[i].joint_name);
      return false;
    }

    if ( std::abs(torque_ff) > tolerance * jointTorqueLimits[i] )
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] torqueSafetyCheck: Torque feedforward command too high for joint " << commands[i].joint_name 
                              << ": torque " << torque_ff
                              << ", tolerance " << tolerance
                              << ", limit " << jointTorqueLimits[i]);
      return false;
    }

    float torque = commands[i].torque_ff + commands[i].kp * (commands[i].desired_position - jointPositions[i]) +
                   commands[i].kd * (commands[i].desired_velocity - jointVelocities[i]);

    if (std::isnan(torque))
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] torqueSafetyCheck: NaN in full torque command for joint " << commands[i].joint_name);
      return false;
    }

    if ( std::abs(torque) > tolerance * jointTorqueLimits[i] )
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] torqueSafetyCheck: Full torque command too high for joint " << commands[i].joint_name 
                              << ": torque " << torque
                              << ", tolerance " << tolerance
                              << ", limit " << jointTorqueLimits[i]);
      return false;
    }
  }

  return true;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_PerceptiveLocomotionOCS2::checkTransition() 
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] checkTransition started \n");

  // Get the next state
  iter++;

  // Switch FSM control mode
  if (this->locomotionSafe() && this->perceptiveOCS2Safe) 
  {
    switch ((int)this->_data->userParameters->control_mode) 
    {
      case FSM_StateName::PERCEPTIVELOCOMOTIONOCS2:
        break;

      case FSM_StateName::PASSIVE:
        this->nextStateName = FSM_StateName::PASSIVE;
        break;

      case FSM_StateName::RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        break;

      default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << (int)FSM_StateName::PERCEPTIVELOCOMOTIONOCS2 << " to "
                  << this->_data->userParameters->control_mode << std::endl;
    }
  } else 
  {
    this->_data->_highcmd->mode = 0;
    this->nextStateName = FSM_StateName::PASSIVE;
  }

  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] checkTransition ended \n");

  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
TransitionData FSM_State_PerceptiveLocomotionOCS2::transition() 
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] transition started");

  // Switch FSM control mode
  switch (this->nextStateName) 
  {
    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] transition ended");

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
void FSM_State_PerceptiveLocomotionOCS2::onExit() 
{
  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] onExit started");

  // Nothing to clean up when exiting
  mpc_running_ = false;
  iter = 0;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM PERCEPTIVE LOCOMOTION OCS2] onExit finished");

  return;
}

} // namespace go2_system
} // namespace legged_software

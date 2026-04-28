#include <ocs2_anymal_commands/AnymalCmdVelToRefTrajPublisher.h>

using namespace ocs2;
using namespace quadruped;


/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
LocalTerrainEstimator::LocalTerrainEstimator(const rclcpp::Node::SharedPtr &node) 
{
    // Load robot description - urdf
    const std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::string urdfString = anymal::getUrdfString(urdfFile);

    // load frame declaration file
    const std::string frameFile = node->get_parameter("frameFile").as_string();

    // Load kinematics model
    kinematicsModel_ = getAnymalKinematics(anymal::frameDeclarationFromFile(frameFile), urdfString);

    lastFootholds_.resize(4);
    for (size_t i = 0; i < 4; i++) 
    {
        lastFootholds_[i] = vector3_t::Zero();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void LocalTerrainEstimator::updateFootholds(const ocs2::SystemObservation &observation) 
{
    // Base position
    auto basePose = getBasePose(observation.state);

    // Joint positions
    auto jointPositions = getJointPositions(observation.state);

    // Compute forward kinematics
    auto footholds = kinematicsModel_->feetPositionsInOriginFrame(basePose, jointPositions);

    // contact flags
    contact_flag_t contactFlags = modeNumber2StanceLeg(observation.mode);

    // Update last footholds
    for (size_t i = 0; i < 4; i++) 
    {
        if (contactFlags[i]) 
        {
            lastFootholds_[i] = footholds[i];
        }
    }
    
    updateLocalTerrainEstimate(lastFootholds_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void LocalTerrainEstimator::updateLocalTerrainEstimate(const std::vector<vector3_t> &footholds) 
{
    const auto normalAndPosition = estimatePlane(footholds);
    terrainPlane_ = TerrainPlane(normalAndPosition.position,
                                 orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(const rclcpp::Node::SharedPtr &node,
                                                            const std::string &targetCommandFile)
{
    node_ = node;
    targetCommandFile_ = targetCommandFile;

    localTerrainEstimator_ = std::make_shared<LocalTerrainEstimator>(node_);

    defaultJointState_.setZero(12);
    loadSettings(targetCommandFile);

    // Subscribe to command velocity
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, std::bind(&ReferenceTrajectoryGenerator::cmdVelCallback, this, std::placeholders::_1));

    // Subscribe to observation
    observation_sub_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
        "/anymal_mpc_observation", 1, std::bind(&ReferenceTrajectoryGenerator::observationCallback, this, std::placeholders::_1));

    // Initialize the reference trajectory publisher
    referencePublisher_ = node->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
        "/anymal_mpc_target", 1);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::loadSettings(const std::string &targetCommandFile) 
{
    // Load target COM height
    ocs2::loadData::loadCppDataType<scalar_t>(targetCommandFile, "comHeight", comHeight_);

    // Load default joint angles
    ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState_);
}

void ReferenceTrajectoryGenerator::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);
    latest_cmd_vel_ = *msg;
}

void ReferenceTrajectoryGenerator::observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(latest_observation_mutex_);
    latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);

    localTerrainEstimator_->updateFootholds(latest_observation_);

    firstObservationReceived_ = true;
}

void ReferenceTrajectoryGenerator::publishReferenceTrajectory()
{
    // std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);
    // std::lock_guard<std::mutex> obs_lock(latest_observation_mutex_);

    if (!firstObservationReceived_) 
    {
        // RCLCPP_WARN_STREAM(node_->get_logger(), "No observation received yet. Cannot publish reference trajectory.");
        return;
    }

    // Generate reference trajectory
    ocs2::TargetTrajectories referenceTrajectory = generateReferenceTrajectory(node_->get_clock()->now().seconds(), 0.0);

    // Publish reference trajectory
    referencePublisher_->publish(ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(referenceTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceHorizon ReferenceTrajectoryGenerator::getBaseReferenceHorizon() 
{
    return {trajdt_, trajKnots_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceCommand ReferenceTrajectoryGenerator::getBaseReferenceCommand(scalar_t time) 
{
    std::lock_guard<std::mutex> lock(latest_cmd_vel_mutex_);

    return {latest_cmd_vel_.linear.x, latest_cmd_vel_.linear.y, latest_cmd_vel_.angular.z, comHeight_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceState ReferenceTrajectoryGenerator::getBaseReferenceState() 
{
    std::lock_guard<std::mutex> lock(latest_observation_mutex_);
    scalar_t observationTime = latest_observation_.time;
    Eigen::Vector3d positionInWorld = latest_observation_.state.segment<3>(3);
    Eigen::Vector3d eulerXyz = latest_observation_.state.head<3>();
    return {observationTime, positionInWorld, eulerXyz};
}

TerrainPlane ReferenceTrajectoryGenerator::getTerrainPlane() 
{
    return localTerrainEstimator_->getPlane();
}


ocs2::TargetTrajectories ReferenceTrajectoryGenerator::generateReferenceTrajectory(scalar_t time, scalar_t dt) 
{
    // Get base reference trajectory
    BaseReferenceTrajectory baseReferenceTrajectory = 
                                generateExtrapolatedBaseReference(getBaseReferenceHorizon(), getBaseReferenceState(),
                                                                    getBaseReferenceCommand(time), getTerrainPlane());

    // Generate target trajectory
    ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
    const size_t N = desiredTimeTrajectory.size();
    ocs2::vector_array_t desiredStateTrajectory(N);
    ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));
    
    for (size_t i = 0; i < N; ++i) 
    {
        ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

        // base orientation
        state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

        auto Rt = switched_model::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

        // base position
        state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

        // base angular velocity
        state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

        // base linear velocity
        state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

        // joint angles
        state.segment<12>(12) = defaultJointState_;

        desiredStateTrajectory[i] = std::move(state);
    }

    ocs2::TargetTrajectories targetTrajectories = {std::move(desiredTimeTrajectory), 
                                                    std::move(desiredStateTrajectory), 
                                                    std::move(desiredInputTrajectory)};

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Generating reference trajectory with " << N << " knots.");
    // for (size_t i = 0; i < N; ++i) 
    // {
    //     const auto& time = targetTrajectories.timeTrajectory[i];
    //     const auto& state = targetTrajectories.stateTrajectory[i];
    //     const auto& input = targetTrajectories.inputTrajectory[i];

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "   time[" << i << "]: " << time);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "   state[" << i << "]");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         torso pose: " << state.head(6).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         torso momentum: " << state.segment(6, 6).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 0: " << state.segment(12, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 1: " << state.segment(15, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 2: " << state.segment(18, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         leg 3: " << state.segment(21, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "   input[" << i << "]: ");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 0 contact force: " << input.segment(0, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 1 contact force: " << input.segment(3, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 2 contact force: " << input.segment(6, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 3 contact force: " << input.segment(9, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 0 joint velocity: " << input.segment(12, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 1 joint velocity: " << input.segment(15, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 2 joint velocity: " << input.segment(18, 3).transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "       leg 3 joint velocity: " << input.segment(21, 3).transpose());
    // }

    return targetTrajectories;
}        

int main(int argc, char* argv[])
{
  const std::string robot_name = "anymal";

  // ros node handle
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robot_name + "_cmd_vel_to_ref_traj_node",
                                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                                .allow_undeclared_parameters(true));

  const std::string targetCommandFile = node->get_parameter("targetCommandFile").as_string();

  ReferenceTrajectoryGenerator ref_traj_gen(node, targetCommandFile);

  rclcpp::Rate rate(5.0);
  while (rclcpp::ok())
  {
    ref_traj_gen.publishReferenceTrajectory();

    rclcpp::spin_some(node);
    rate.sleep();
  }

  // ros::spin();
  // Successful exit
  return 0;
}
#include <mmp_quadruped/environment/EnvironmentProcessor.h>


namespace mmp {

EnvironmentProcessor::EnvironmentProcessor(const rclcpp::Node::SharedPtr& node,
                                            std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface, 
                                            const std::string & envFile,
                                            base_coordinate_t & torsoInitPos,
                                            base_coordinate_t & torsoGlobalGoalPose)
{
    node_ = node;
    RCLCPP_INFO_STREAM(node_->get_logger(), "[EnvironmentProcessor()]");

    interface_ = interface;

    envFile_ = envFile;

    minEnvPt << 1e30, 1e30, 1e30;
    maxEnvPt << -1e30, -1e30, -1e30;

    // set torso global goal pose
    setTorsoGlobalGoalPose(torsoInitPos, torsoGlobalGoalPose);    
}

EnvironmentProcessor::~EnvironmentProcessor()
{
}

// void EnvironmentProcessor::setRegions()

bool EnvironmentProcessor::readyToPlan() 
{ 
    RCLCPP_INFO_STREAM(node_->get_logger(), "      convex terrains size: " << interface_->getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner().getTerrainModel().getAllConvexTerrains().size());

    allRegions_ = interface_->getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner().getTerrainModel().getAllConvexTerrains(); 
 
    return !allRegions_.empty(); 
}


// void EnvironmentProcessor::callback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstPtr& msg)
// {
//     // Read terrain
//     auto terrainPtr = std::make_unique<switched_model::SegmentedPlanesTerrainModel>(convex_plane_decomposition::fromMessage(*msg));
//     terrainPtr_.swap(terrainPtr);
// }

// void EnvironmentProcessor::parseEnvFile(const Eigen::VectorXd & latest_q)
// {
//     parseStoneFile(latest_q);
// }

void EnvironmentProcessor::setTorsoGlobalGoalPose(base_coordinate_t & torsoInitPos, base_coordinate_t & torsoGlobalGoalPose)
{
    // TODO: just change this into a world frame / robot frame desired position
    //      Then, can just read in from Rviz
    RCLCPP_INFO_STREAM(node_->get_logger(), "[setTorsoGlobalGoalPose()]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "  envFile_: " << envFile_);

    if (envFile_.find("/empty.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 3.0, 0.0, 0.300;
    } else if (envFile_.find("/balance_beam.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -3.0, 0.0, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 2.0, 0.0, 0.300;
    } else if (envFile_.find("/offset_beam.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -3.0, 0.75, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 2.0, 0.75, 0.300;
    } else if (envFile_.find("/pegboard.json") != std::string::npos)
    {
        torsoInitPos << 0.0 , 0.0, 0.0, -2.5, 1.4, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 3.5, 1.4, 0.300; 
    } else if (envFile_.find("/ramp_10.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0,  -3.0, 0.0, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0,  2.5, 0.0, 0.825;   
    } else if (envFile_.find("/ramped_balance_beam.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, -M_PI, 0.0, 0.50 , 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, -M_PI, -8.50, 0.50, 0.300;
    } else if (envFile_.find("/ramped_stepping_stones.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, -M_PI, 0.50, 7.50 , 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, -M_PI, -8.0, 7.50, 0.300;
    } else if (envFile_.find("/ramped_balance_beam_easy.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, -M_PI, 0.0, 0.50 , 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, -M_PI, -8.50, 0.50, 0.300;
    } else if (envFile_.find("/ramped_stepping_stones_easy.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, -M_PI, 0.50, 7.50 , 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, -M_PI, -8.0, 7.50, 0.300;    
    } else if (envFile_.find("/rubble.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -2.5, 1.4, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 3.5, 1.4, 0.300;
    } else if (envFile_.find("/side_stones.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -2.5, 0.15, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 2.0, 0.15, 0.300;          
    } else if (envFile_.find("/sparse_stones.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -3.0, 1.4, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 2.0, 1.4, 0.300;    
    } else if (envFile_.find("/stairs.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -2.5, 0.0, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 3.0, 0.0, 0.875;     
    } else if (envFile_.find("/down_ramp.json") != std::string::npos)
    {
        torsoInitPos << 0.0, 0.0, 0.0, -0.70, 0.0, 0.300;
        torsoGlobalGoalPose << 0.0, 0.0, 0.0, 6.50, 0.0, -0.900;
    } else
    {
        throw std::runtime_error("[setTorsoGlobalGoalPose() -- need to add env file: " + envFile_ + " to list for torsoGlobalGoalPose]");
    }
}

// 
//                         
// const geometry_msgs::msg::TransformStamped & worldFrameToBaseFrameTransform                      
void EnvironmentProcessor::extractLocalRegions(const comkino_state_t & latest_x)
{
    // Eigen::VectorXd latest_q = latest_x.tail<CONFIG_DIM>(); // copying configuration from state

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    vector3_t torso_position = extractTorsoPosition(torso_pose);

    RCLCPP_INFO_STREAM(node_->get_logger(), "[extractLocalRegions()]");

    RCLCPP_INFO_STREAM(node_->get_logger(), "      torso_position: " << torso_position.transpose());

    // const auto& planarRegions = terrainPtr_->planarTerrain().planarRegions;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      grabbed planarRegions ");

    localRegions_.clear();
    resetMinMaxEnvPts();

    // extracting local regions
    for (const auto & region : allRegions_)
    {
        vector3_t regionNominalPositionWorldFrame = region.plane.positionInWorld.transpose();

        updateMinMaxEnvPts(regionNominalPositionWorldFrame);
        
        // push back region
        localRegions_.push_back(region);
    } 

    // add four regions for starting feet positions as last resorts
    const joint_coordinate_t qJoints = getJointPositions(latest_x);
    for (int leg = 0; leg < 4; leg++)
    {
        vector3_t foot_posn = interface_->getKinematicModel().footPositionInOriginFrame(leg, torso_pose, qJoints);
        updateMinMaxEnvPts(foot_posn);

        switched_model::ConvexTerrain footRegion;
        footRegion.plane.positionInWorld = foot_posn;
        footRegion.plane.orientationWorldToTerrain = Eigen::Matrix3d::Identity();
        footRegion.boundary.push_back(vector2_t(FOOT_RADIUS, FOOT_RADIUS));
        footRegion.boundary.push_back(vector2_t(-FOOT_RADIUS, FOOT_RADIUS));
        footRegion.boundary.push_back(vector2_t(-FOOT_RADIUS, -FOOT_RADIUS));
        footRegion.boundary.push_back(vector2_t(FOOT_RADIUS, -FOOT_RADIUS));
        localRegions_.push_back(footRegion);
    }
}

void EnvironmentProcessor::updateMinMaxEnvPts(const vector3_t & endpt1, const vector3_t & endpt2)
{
    updateMinMaxEnvPts(endpt1);
    updateMinMaxEnvPts(endpt2);
}

void EnvironmentProcessor::updateMinMaxEnvPts(const vector3_t & pt)
{
    for (int i = 0; i < pt.size(); i++)
    {
        if (pt[i] < minEnvPt[i])
            minEnvPt[i] = pt[i];
        if (pt[i] > maxEnvPt[i])
            maxEnvPt[i] = pt[i];
    }
}

}  // namespace mmp

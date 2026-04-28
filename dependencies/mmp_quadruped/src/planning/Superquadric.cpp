#include <mmp_quadruped/planning/Superquadric.h>


namespace mmp {

Superquadric::Superquadric(const rclcpp::Node::SharedPtr& node,
                            const int & leg_idx, 
                            const std::string & hyperparametersFile)
{
    node_ = node;

    double sqDimX = 0.0;
    double sqDimY = 0.0;
    double sqDimZ = 0.0;
    double sqCurvatureX = 0.0;
    double sqCurvatureY = 0.0;
    double sqCurvatureZ = 0.0;
    double x_offset_front = 0.0; // 0.225;
    double x_offset_back = 0.0; // -0.225;
    double y_offset_front = 0.0;
    double y_offset_back = 0.0;
    double z_offset_front = 0.0; // 0.0;
    double z_offset_back = 0.0;
    double roll_front = 0.0;
    double roll_back = 0.0;
    double pitch_front = 0.0;
    double pitch_back = 0.0;
    double yaw_front = 0.0;
    double yaw_back = 0.0;

    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(hyperparametersFile, pt);

    const std::string sqPrefix = "superquadrics.";
    ocs2::loadData::loadPtreeValue(pt, sqDimX, sqPrefix + "dimX", verbose);
    ocs2::loadData::loadPtreeValue(pt, sqDimY, sqPrefix + "dimY", verbose);
    ocs2::loadData::loadPtreeValue(pt, sqDimZ, sqPrefix + "dimZ", verbose);
    ocs2::loadData::loadPtreeValue(pt, sqCurvatureX, sqPrefix + "curvX", verbose);
    ocs2::loadData::loadPtreeValue(pt, sqCurvatureY, sqPrefix + "curvY", verbose);
    ocs2::loadData::loadPtreeValue(pt, sqCurvatureZ, sqPrefix + "curvZ", verbose);   

    ocs2::loadData::loadPtreeValue(pt, x_offset_front, sqPrefix + "x_offset_front", verbose);
    ocs2::loadData::loadPtreeValue(pt, x_offset_back, sqPrefix + "x_offset_back", verbose);
    ocs2::loadData::loadPtreeValue(pt, y_offset_front, sqPrefix + "y_offset_front", verbose);
    ocs2::loadData::loadPtreeValue(pt, y_offset_back, sqPrefix + "y_offset_back", verbose);
    ocs2::loadData::loadPtreeValue(pt, z_offset_front, sqPrefix + "z_offset_front", verbose);
    ocs2::loadData::loadPtreeValue(pt, z_offset_back, sqPrefix + "z_offset_back", verbose);
    ocs2::loadData::loadPtreeValue(pt, roll_front, sqPrefix + "roll_front", verbose);
    ocs2::loadData::loadPtreeValue(pt, roll_back, sqPrefix + "roll_back", verbose);
    ocs2::loadData::loadPtreeValue(pt, pitch_front, sqPrefix + "pitch_front", verbose);
    ocs2::loadData::loadPtreeValue(pt, pitch_back, sqPrefix + "pitch_back", verbose);
    ocs2::loadData::loadPtreeValue(pt, yaw_front, sqPrefix + "yaw_front", verbose);
    ocs2::loadData::loadPtreeValue(pt, yaw_back, sqPrefix + "yaw_back", verbose);


    // sqCurvature << sqCurvatureX, sqCurvatureY, sqCurvatureZ;
    // sqDimension << sqDimX, sqDimY, sqDimZ;

    if (leg_idx == FL)
    {
        sqDimension << sqDimX, sqDimY, sqDimZ;
        sqCurvature << sqCurvatureX, sqCurvatureY, sqCurvatureZ;
        sqCenter << x_offset_front, y_offset_front, z_offset_front;
        sqOrientation << roll_front, pitch_front, yaw_front;
    } else if (leg_idx == FR)
    {
        sqDimension << sqDimX, sqDimY, sqDimZ;
        sqCurvature << sqCurvatureX, sqCurvatureY, sqCurvatureZ;
        sqCenter << x_offset_front, -y_offset_front, z_offset_front;
        sqOrientation << -roll_front, pitch_front, yaw_front;
    } else if (leg_idx == BL)
    {
        sqDimension << sqDimX, sqDimY, sqDimZ;
        sqCurvature << sqCurvatureX, sqCurvatureY, sqCurvatureZ;
        sqCenter << x_offset_back, y_offset_back, z_offset_back;
        sqOrientation << roll_back, pitch_back, yaw_back;
    } else if (leg_idx == BR)
    {
        sqDimension << sqDimX, sqDimY, sqDimZ;
        sqCurvature << sqCurvatureX, sqCurvatureY, sqCurvatureZ;
        sqCenter << x_offset_back, -y_offset_back, z_offset_back;
        sqOrientation << -roll_back, pitch_back, yaw_back;
    } else
    {
        throw std::invalid_argument("Invalid leg index.");
    }

    // superquadricToRbtFrame = Eigen::Matrix4d::Identity();
    // superquadricToRbtFrame.block<3,3>(0,0) = calculateRotationMatrix(roll, pitch, yaw);
    // superquadricToRbtFrame.block<3,1>(0,3) = sqCenter;
    const Eigen::Matrix3d R_superquadric2rbt = rotationMatrixBaseToOrigin<scalar_t>(sqOrientation);
    Eigen::Matrix3d R_rbt2superquadric = R_superquadric2rbt.transpose();

    rbtToSuperquadricFrame = Eigen::Matrix4d::Identity();
    rbtToSuperquadricFrame.block<3,3>(0,0) = R_rbt2superquadric; // rotMatTranspose; 
    rbtToSuperquadricFrame.block<3,1>(0,3) = -R_rbt2superquadric * sqCenter; // 

    // rbtToSuperquadricFrame = superquadricToRbtFrame.inverse();
}

bool Superquadric::isRegionWithinSuperquadricWorldFrame(const base_coordinate_t & torso_pose, 
                                                        const switched_model::ConvexTerrain & region,
                                                        const bool & print)
{
    if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "[isRegionWithinSuperquadricWorldFrame()]");

    // bool isWithin = true;

    // Check center point
    vector3_t p_center_world_frame = region.plane.positionInWorld;

    if (print)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                          p_center_world_frame: " << p_center_world_frame.transpose());
    }

    if (!isPointWithinSuperquadricWorldFrame(torso_pose, p_center_world_frame, print))
    {
        if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "region not within superquadric");
        return false;
    }

    // Check whole boundary
    // for (const auto & p : region.boundary)
    // {
    //     vector3_t p_extended(p.x(), p.y(), 0.0); // terrain frame
    //     vector3_t p_query_world_frame = switched_model::positionInWorldFrameFromPositionInTerrain(p_extended, region.plane);

    //     // if (print)
    //     // {
    //     //     RCLCPP_INFO_STREAM(node_->get_logger(), "p_extended: " << p_extended.transpose());
    //     //     RCLCPP_INFO_STREAM(node_->get_logger(), "region.plane.positionInWorld: " << region.plane.positionInWorld.transpose());
    //     //     RCLCPP_INFO_STREAM(node_->get_logger(), "region.plane.orientationWorldToTerrain: ");
    //     //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                        " << region.plane.orientationWorldToTerrain);

    //     //     RCLCPP_INFO_STREAM(node_->get_logger(), "p_query_world_frame: " << p_query_world_frame.transpose());
    //     // }

    //     if (!isPointWithinSuperquadricWorldFrame(torso_pose, p_query_world_frame, print))
    //     {
    //         if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "region not within superquadric");
    //         return false;
    //     }
    // }

    if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "region is within superquadric");


    return true;
}

// For if we are using world frame
bool Superquadric::isPointWithinSuperquadricWorldFrame(const base_coordinate_t & torso_pose_world_frame, 
                                                        const vector3_t & p_query_world_frame,
                                                        const bool & print)
{
    if (print) RCLCPP_INFO_STREAM(node_->get_logger(), "                          [pointWithinSuperquadric()]");

    vector3_t p_torso_world_frame = extractTorsoPosition(torso_pose_world_frame);
    vector3_t orient_torso_world_frame = extractTorsoOrientation(torso_pose_world_frame);

    if (print)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          p_torso_world_frame: " << p_torso_world_frame.transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          orient_torso_world_frame: " << orient_torso_world_frame.transpose());
    }

    // double total_roll = orient_torso_world_frame[0] + sqOrientation[0];
    // double total_pitch = orient_torso_world_frame[1] + sqOrientation[1];
    // double total_yaw = orient_torso_world_frame[2] + sqOrientation[2];

    // double roll = orient_torso_world_frame[0];
    // double pitch = orient_torso_world_frame[1];
    // double yaw = orient_torso_world_frame[2];

    // if (print)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          roll: " << roll);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          pitch: " << pitch);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          yaw: " << yaw);
    // }

    // Eigen::Matrix3d R_rbt2world = calculateRotationMatrix(roll, pitch, yaw);

    const Eigen::Matrix3d R_rbt2world = rotationMatrixBaseToOrigin<scalar_t>(orient_torso_world_frame);

    // rbt2world << R11, R12, R13, p_torso[0],
    //              R21, R22, R23, p_torso[1],
    //              R31, R32, R33, p_torso[2],
    //              0.0, 0.0, 0.0, 1.0;

    std::chrono::steady_clock::time_point timeBegin, timeEnd;

    timeBegin = std::chrono::steady_clock::now();

    // Eigen::Matrix4d rbt2world = Eigen::Matrix4d::Identity();
    // rbt2world.block<3,3>(0,0) = rotMat;
    // rbt2world.block<3,1>(0,3) = p_torso;

    // Eigen::Matrix4d world2rbt = rbt2world.inverse(); // TODO: how not to inverse every time
    // timeEnd = std::chrono::steady_clock::now();

    // int64_t timeElapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeBegin).count();
    // RCLCPP_INFO_STREAM(node_->get_logger(), " inverse time (us): " << timeElapsed);

    // timeBegin = std::chrono::steady_clock::now();

    Eigen::Matrix4d world2rbt = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_world2rbt = R_rbt2world.transpose();
    world2rbt.block<3,3>(0,0) = R_world2rbt; // rotMatTranspose; 
    world2rbt.block<3,1>(0,3) = -R_world2rbt * p_torso_world_frame; // 

    // timeEnd = std::chrono::steady_clock::now();

    // timeElapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeBegin).count();
    // RCLCPP_INFO_STREAM(node_->get_logger(), " test time (us): " << timeElapsed);    


    Eigen::Vector4d aff_p_query;
    aff_p_query << p_query_world_frame, 1.0;
    // proj_p_query[2] = 1.0;

    Eigen::Vector4d pQueryAffRbtFrame = world2rbt * aff_p_query;
    Eigen::Vector4d pQueryAffSuperquadricFrame = rbtToSuperquadricFrame * pQueryAffRbtFrame;
    vector3_t pQueryRbtFrame = pQueryAffRbtFrame.head<3>();
    vector3_t pQuerySuperquadricFrame = pQueryAffSuperquadricFrame.head<3>();

    // vector3_t pQueryRbtFrame = pQueryAffRbtFrame.head(3);

    // vector3_t pQueryWorldFrame = p_query_world_frame;


    // Eigen::Vector4d pQueryAffRbtFrameTest = world2rbtTest * aff_p_query;
    // vector3_t pQueryRbtFrameTest = pQueryAffRbtFrameTest.head(3);

    if (print)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          world2rbt: ");
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                                     " << world2rbt);

        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          p_query_world_frame: " << p_query_world_frame.transpose());

        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          pQueryRbtFrame: " << pQueryRbtFrame.transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          pQuerySuperquadricFrame: " << pQuerySuperquadricFrame.transpose());

        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          sqCenter: " << sqCenter.transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          sqCurvature: " << sqCurvature.transpose());
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          sqDimension: " << sqDimension.transpose());
    }

    // Eigen::Matrix2d rotMat;
    // rotMat << std::cos(latest_q[3]), -std::sin(latest_q[3]),
    //           std::sin(latest_q[3]),  std::cos(latest_q[3]);  

    // RCLCPP_INFO_STREAM(node_->get_logger(), "robot yaw: " << latest_q[3]);

    // vector3_t sqOrigin = sqCenters[leg_idx]; // p_torso.head(2) + rotMat * 
    // vector3_t sqCurvature = sqCurvatures[leg_idx]; // rotMat * 
    // vector3_t sqDimension = sqDimensions[leg_idx]; // rotMat * 

    // RCLCPP_INFO_STREAM(node_->get_logger(), "                                          p_query: " << p_query.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "                                          pQueryRbtFrame: " << pQueryRbtFrame.transpose());

    // in the future: transform queried point into robot frame and evaluate there

    // Eigen::Vector2d sqOrigin(p_torso[0] + x0s[leg_idx],
    //                                    p_torso[1] + y0s[leg_idx]); 


    // double value = (std::pow(std::abs((pQueryRbtFrame[0]) / sqDimension[0]), sqCurvature[0]) + 
                    // std::pow(std::abs((pQueryRbtFrame[1]) / sqDimension[1]), sqCurvature[1]) +
                    // std::pow(std::abs((pQueryRbtFrame[2]) / sqDimension[2]), sqCurvature[2]));

    // double value = (std::pow(std::abs((pQueryRbtFrame[0] - sqCenter[0]) / sqDimension[0]), sqCurvature[0]) + 
    //                 std::pow(std::abs((pQueryRbtFrame[1] - sqCenter[1]) / sqDimension[1]), sqCurvature[1]) +
    //                 std::pow(std::abs((pQueryRbtFrame[2] - sqCenter[2]) / sqDimension[2]), sqCurvature[2]));

    double value = (std::pow(std::abs((pQuerySuperquadricFrame[0]) / sqDimension[0]), sqCurvature[0]) + 
                    std::pow(std::abs((pQuerySuperquadricFrame[1]) / sqDimension[1]), sqCurvature[1]) +
                    std::pow(std::abs((pQuerySuperquadricFrame[2]) / sqDimension[2]), sqCurvature[2]));

    // double valueTest = (std::pow(std::abs((pQueryRbtFrameTest[0] - sqCenter[0]) / sqDimension[0]), sqCurvature[0]) + 
    //                     std::pow(std::abs((pQueryRbtFrameTest[1] - sqCenter[1]) / sqDimension[1]), sqCurvature[1]) +
    //                     std::pow(std::abs((pQueryRbtFrameTest[2] - sqCenter[2]) / sqDimension[2]), sqCurvature[2]));
    // if (std::abs(orient_torso[0]) < 0.001 && std::abs(orient_torso[2]) < 0.001)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          leg_idx: " << leg_idx);

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          p_torso: " << p_torso.transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          orient_torso: " << orient_torso.transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          orient_torso: " << orient_torso.transpose());

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          sqOrigin: " << sqOrigin.transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          sqCurvature: " << sqCurvature.transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          sqDimension: " << sqDimension.transpose());

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          pQueryRbtFrame: " << pQueryRbtFrame.transpose());
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                                          value: " << value);

    // }

    if (print)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "                                          value: " << value);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "                                          valueTest: " << valueTest);
    }
    // if ( (value <= 1.0) != (valueTest <= 1.0) )
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), " different results!");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "      value: " << value);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "      valueTest: " << valueTest);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "      world2rbt: " << world2rbt);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "      world2rbtTest: " << world2rbtTest);

    //     throw std::runtime_error("Different results!");
    // }

    return (value <= 1.0);
}

}  // namespace mmp
 
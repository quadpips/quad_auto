// #include <pinocchio/fwd.hpp> // NEED TO KEEP THIS AT TOP

// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/model.hpp"

#include <mmp_quadruped/planning/ConstraintManager.h>

namespace mmp {

ConstraintManager::ConstraintManager(const rclcpp::Node::SharedPtr& node,
                                        std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface)
{
    node_ = node;
    positions.resize(4);

    J_FL = Eigen::MatrixXd::Zero(6, 12);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "after J_FL");
    J_FR = Eigen::MatrixXd::Zero(6, 12);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "after J_FR");
    J_BL = Eigen::MatrixXd::Zero(6, 12);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "after J_BL");
    J_BR = Eigen::MatrixXd::Zero(6, 12);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "after J_BR");

    J_total = Eigen::MatrixXd::Zero(12, 12); 

    interface_ = interface;

}

void ConstraintManager::resetConfigurations()
{
    current_x = start_x;
    current_u = start_u;
    target_x = start_x;
    target_u = start_u;
}

void ConstraintManager::enforceJointLimits(Eigen::VectorXd & q,
                                            const vector3_t & contactCenter,
                                            const vector3_t & localContactNormal)
{
    // need to artificially set z height constraint
    for (int i = 0; i < CONFIG_DIM; i++) // 18
    {
        if (i >= 0 && i < 3) // robot torso orientation
        {
            // unwind
            while (q[i] < -2.0 * M_PI) 
                q[i] += 2.0 * M_PI;

            while (q[i] > 2.0 * M_PI) 
                q[i] -= 2.0 * M_PI;  

            // clamp with limits
            // q[i] = std::max(model.lowerPositionLimit[i], std::min(q[i], model.upperPositionLimit[i]));  

        } else if (i >= 3 && i < 6) // robot torso position
        {
            // robot x position

            // robot y position

            if (i == 5) // robot z position
            {
                vector3_t pMin = contactCenter + localContactNormal * (0.9 * Z_TORSO_OFFSET); 
                vector3_t pMax = contactCenter + localContactNormal * (1.1 * Z_TORSO_OFFSET); 
                double z_min = pMin[2];
                double z_max = pMax[2];
                q[i] = std::max(z_min, std::min(q[i], z_max));
            }
            
        } else // joint angles
        {
            // unwind
            while (q[i] < -2.0 * M_PI) 
                q[i] += 2.0 * M_PI;

            while (q[i] > 2.0 * M_PI) 
                q[i] -= 2.0 * M_PI;         

            // clamp with limits
            int offset = 6;
            q[i] = std::max(interface_->modelSettings().lowerJointLimits_[i - offset], 
                                std::min(q[i], interface_->modelSettings().upperJointLimits_[i - offset]));
        }
    }
}

bool ConstraintManager::findTransitionConfiguration()
{
    updateLegsPositionConstraints();
    return findConfiguration(target_x);
}

bool ConstraintManager::findConfiguration(comkino_state_t & incoming_x)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[findConfiguration]");
    int num_sample_iterations = 25;

    comkino_state_t defaultState = interface_->getInitialState();

    std::default_random_engine generator;
    std::uniform_real_distribution<double> com_xy_distribution(-0.075, 0.075);
    std::uniform_real_distribution<double> com_z_distribution(-0.05, 0.05);
    std::uniform_real_distribution<double> joint_distribution(-M_PI/8, M_PI/8);
    
    bool foundTransition = false;

    vector3_t contactCenter = (positions[FL] + positions[FR] + positions[BL] + positions[BR]) / 4.0;

    float referenceYaw = incoming_x[2];
    vector3_t localContactNormal = estimatePlaneNormal(positions);
    vector3_t torsoOrientation = estimateEulerAnglesFromContacts(positions, referenceYaw);

    RCLCPP_INFO_STREAM(node_->get_logger(), "      incoming_x: " << incoming_x.transpose());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      localContactNormal: " << localContactNormal.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      torsoOrientation: " << torsoOrientation.transpose());

    // for IK, extracting just the configuration
    Eigen::VectorXd new_q = Eigen::VectorXd::Zero(CONFIG_DIM);

    for (int i = 0; i < num_sample_iterations; i++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "projection " << i);

        new_q.segment(0, 3) = torsoOrientation;
        new_q.segment(3, 3) = contactCenter + localContactNormal * Z_TORSO_OFFSET;
        new_q.tail(12) = defaultState.block(12, 0, 12, 1);

        // for first iteration, just check if configuration is already satisfying the contact constraint.
        if (i > 0)
        {
            // perturb the torso position
            new_q[3] += com_xy_distribution(generator);
            new_q[4] += com_xy_distribution(generator);
            new_q[5] += com_z_distribution(generator);
        }

        // RCLCPP_INFO_STREAM(node_->get_logger(), "      new_q: " << new_q.transpose());

        foundTransition = IKProjection(new_q, contactCenter, localContactNormal);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "  foundTransition: " << foundTransition);

        if (foundTransition)
            break;
    }

    incoming_x.head(6) = getBasePose(new_q); // .head(6); // torso pose
    incoming_x.segment(6, 6) = Eigen::VectorXd::Zero(6); // momentum
    incoming_x.tail(12) = new_q.tail(12); // joint angles

    RCLCPP_INFO_STREAM(node_->get_logger(), "      resulting incoming_x: " << incoming_x.transpose());

    return foundTransition;
}

bool ConstraintManager::checkModeFamilyConstraint(const comkino_state_t & latest_x,
                                                    ModeFamily * destModeFamily)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[checkModeFamilyConstraint()]");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      latest_x: ");
    RCLCPP_INFO_STREAM(node_->get_logger(), stateString(latest_x));

    bool destModeFamilyConstraintMet = true;
    for (int leg = 0; leg < 4; leg++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "leg: " << leg);

        if (!destModeFamily->isLegInSwing(leg))
        {        
            // need to check goal mode family (all feet in contact)

            const base_coordinate_t basePose = getBasePose(latest_x);
            const joint_coordinate_t qJoints = getJointPositions(latest_x);

            // RCLCPP_INFO_STREAM(node_->get_logger(), "      foot position: " << interface_->getKinematicModel().footPositionInOriginFrame(leg, basePose, qJoints).transpose());

            if (!destModeFamily->isFootholdOnRegionWorldFrame(leg, interface_->getKinematicModel().footPositionInOriginFrame(leg, 
                                                                                                                            basePose, 
                                                                                                                            qJoints))) // data.oMf[foot_frame_id].translation()))
            {
                destModeFamilyConstraintMet = false;
                // RCLCPP_INFO_STREAM(node_->get_logger(), "      failed!");
            } else
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "      passed!");
            }
        } else
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "      leg in swing!");
        }
    }

    return destModeFamilyConstraintMet;    
}

bool ConstraintManager::checkGoalModeFamilyConstraint(const comkino_state_t & latest_x)
{
    return checkModeFamilyConstraint(latest_x, graph_->getGoalModeFamily());
}

bool ConstraintManager::checkGlobalGoalConstraint(const comkino_state_t & latest_x,
                                                    const base_coordinate_t & torsoGlobalGoalPose)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[ConstraintManager::checkGlobalGoalConstraint()]");
    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    vector3_t torso_position = extractTorsoPosition(torso_pose);

    vector3_t torsoGlobalGoalPosition = extractTorsoPosition(torsoGlobalGoalPose); // .head(3);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      torso_position: " << torso_position.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      torsoGlobalGoalPosition: " << torsoGlobalGoalPosition.transpose());
    
    return (torso_position - torsoGlobalGoalPosition).norm() < graph_->getGlobalGoalRegionTolerance();
}

bool ConstraintManager::checkDestinationModeFamilyConstraint()
{
    return checkModeFamilyConstraint(current_x, graph_->getCurrentTransition()->getNodeTo()->modeFamily);
}

// bool ConstraintManager::checkConfiguration()
// {
//     // is hip above knee (in base frame)

//     // is knee above foot (in base frame)
// }

bool ConstraintManager::IKProjection(Eigen::VectorXd & new_q,  
                                        const vector3_t & contactCenter,
                                        const vector3_t & localContactNormal)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[IKProjection:]");

    // RCLCPP_INFO_STREAM(node_->get_logger(), "      new_q: " << new_q.transpose());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  FL_foot target position: " << positions[FL].transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  FR_foot target position: " << positions[FR].transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  BL_foot target position: " << positions[BL].transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  BR_foot target position: " << positions[BR].transpose());    

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  contactCenter: " << contactCenter.transpose());    
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  localContactNormal: " << localContactNormal.transpose());    

    // bool collisionFree = false; 
    bool inContact = false;
    bool foundTransition = false;
    bool isStable = false;

    double epsilon = 1.0e-5;
    int num_projection_iterations = 25;
    Eigen::VectorXd x_base = Eigen::VectorXd::Zero(12); // 3 Dof per foot, constraining position
    Eigen::VectorXd x_world = Eigen::VectorXd::Zero(12); // 3 Dof per foot, constraining position

    for (int projection_iteration = 0; projection_iteration < num_projection_iterations; projection_iteration++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "      IK projection " << projection_iteration);

        const base_coordinate_t basePose = getBasePose(new_q); // .head(6);
        vector3_t torsoOrientation = extractTorsoOrientation(basePose);
        vector3_t torsoPosition = extractTorsoPosition(basePose);
        const joint_coordinate_t qJoints = new_q.tail(12);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          torsoOrientation: " << torsoOrientation.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          torsoPosition: " << torsoPosition.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          qJoints: " << qJoints.transpose());

        vector3_t FL_foot_posn_world = interface_->getKinematicModel().footPositionInOriginFrame(FL, basePose, qJoints);
        vector3_t FR_foot_posn_world = interface_->getKinematicModel().footPositionInOriginFrame(FR, basePose, qJoints);
        vector3_t BL_foot_posn_world = interface_->getKinematicModel().footPositionInOriginFrame(BL, basePose, qJoints);
        vector3_t BR_foot_posn_world = interface_->getKinematicModel().footPositionInOriginFrame(BR, basePose, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_foot_posn_world: " << FL_foot_posn_world.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_foot_posn_world: " << FR_foot_posn_world.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_foot_posn_world: " << BL_foot_posn_world.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_foot_posn_world: " << BR_foot_posn_world.transpose());    

        vector3_t FL_foot_posn_base = interface_->getKinematicModel().positionBaseToFootInBaseFrame(FL, qJoints);
        vector3_t FR_foot_posn_base = interface_->getKinematicModel().positionBaseToFootInBaseFrame(FR, qJoints);
        vector3_t BL_foot_posn_base = interface_->getKinematicModel().positionBaseToFootInBaseFrame(BL, qJoints);
        vector3_t BR_foot_posn_base = interface_->getKinematicModel().positionBaseToFootInBaseFrame(BR, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_foot_posn_base: " << FL_foot_posn_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_foot_posn_base: " << FR_foot_posn_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_foot_posn_base: " << BL_foot_posn_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_foot_posn_base: " << BR_foot_posn_base.transpose());

        // Eigen::Matrix3d FL_foot_orientation_base = interface_->getKinematicModel().footOrientationInBaseFrame(FL, qJoints);
        // vector3_t FL_foot_euler_base = eulerXYZFromRotationMatrix(FL_foot_orientation_base, torsoOrientation[2]);
        // Eigen::Matrix3d FR_foot_orientation_base = interface_->getKinematicModel().footOrientationInBaseFrame(FR, qJoints);
        // vector3_t FR_foot_euler_base = eulerXYZFromRotationMatrix(FR_foot_orientation_base, torsoOrientation[2]);
        // Eigen::Matrix3d BL_foot_orientation_base = interface_->getKinematicModel().footOrientationInBaseFrame(BL, qJoints);
        // vector3_t BL_foot_euler_base = eulerXYZFromRotationMatrix(BL_foot_orientation_base, torsoOrientation[2]);
        // Eigen::Matrix3d BR_foot_orientation_base = interface_->getKinematicModel().footOrientationInBaseFrame(BR, qJoints);
        // vector3_t BR_foot_euler_base = eulerXYZFromRotationMatrix(BR_foot_orientation_base, torsoOrientation[2]);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_foot_euler_base: " << FL_foot_euler_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_foot_euler_base: " << FR_foot_euler_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_foot_euler_base: " << BL_foot_euler_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_foot_euler_base: " << BR_foot_euler_base.transpose());

        // These are slightly different than joint limits (I think)
        // float min_foot_roll = -0.50;
        // float max_foot_roll =  0.50;
        // float min_foot_pitch = -1.1;
        // float max_foot_pitch = -0.20;

        // bool isStable = (FL_foot_euler_base[0] > min_foot_roll && FL_foot_euler_base[0] < max_foot_roll
        //                 && FL_foot_euler_base[1] > min_foot_pitch && FL_foot_euler_base[1] < max_foot_pitch
        //                 && FR_foot_euler_base[0] > min_foot_roll && FR_foot_euler_base[0] < max_foot_roll
        //                 && FR_foot_euler_base[1] > min_foot_pitch && FR_foot_euler_base[1] < max_foot_pitch
        //                 && BL_foot_euler_base[0] > min_foot_roll && BL_foot_euler_base[0] < max_foot_roll
        //                 && BL_foot_euler_base[1] > min_foot_pitch && BL_foot_euler_base[1] < max_foot_pitch
        //                 && BR_foot_euler_base[0] > min_foot_roll && BR_foot_euler_base[0] < max_foot_roll
        //                 && BR_foot_euler_base[1] > min_foot_pitch && BR_foot_euler_base[1] < max_foot_pitch);

        // // world to base frame transform matrix
        // Eigen::Matrix4d baseFrameToWorldFrameTransform;

        // baseFrameToWorldFrameTransform.block<3, 3>(0, 0) = ocs2::getRotationMatrixFromXyzEulerAngles(torsoOrientation);
        // baseFrameToWorldFrameTransform.block<3, 1>(0, 3) = torsoPosition;
        // baseFrameToWorldFrameTransform(3, 3) = 1.0;

        // Eigen::Matrix4d worldFrameToBaseFrameTransform = baseFrameToWorldFrameTransform.inverse();

        // double roll = torsoOrientation[0];
        // double pitch = torsoOrientation[1];
        // double yaw = torsoOrientation[2];
        // Eigen::Matrix3d R_rbt2world = calculateRotationMatrix(roll, pitch, yaw);
        const Eigen::Matrix3d R_rbt2world = rotationMatrixBaseToOrigin<scalar_t>(torsoOrientation);

        Eigen::Matrix4d world2rbt = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R_world2rbt = R_rbt2world.transpose();
        world2rbt.block<3,3>(0,0) = R_world2rbt; // rotMatTranspose; 
        world2rbt.block<3,1>(0,3) = -R_world2rbt * torsoPosition; // 

        Eigen::Vector4d FL_foot_tgt_world(positions[FL][0], positions[FL][1], positions[FL][2], 1.0);
        Eigen::Vector4d FL_foot_tgt_base_hg = world2rbt * FL_foot_tgt_world;
        vector3_t FL_foot_tgt_base = FL_foot_tgt_base_hg.head(3);

        Eigen::Vector4d FR_foot_tgt_world(positions[FR][0], positions[FR][1], positions[FR][2], 1.0);
        Eigen::Vector4d FR_foot_tgt_base_hg = world2rbt * FR_foot_tgt_world;
        vector3_t FR_foot_tgt_base = FR_foot_tgt_base_hg.head(3);

        Eigen::Vector4d BL_foot_tgt_world(positions[BL][0], positions[BL][1], positions[BL][2], 1.0);
        Eigen::Vector4d BL_foot_tgt_base_hg = world2rbt * BL_foot_tgt_world;
        vector3_t BL_foot_tgt_base = BL_foot_tgt_base_hg.head(3);

        Eigen::Vector4d BR_foot_tgt_world(positions[BR][0], positions[BR][1], positions[BR][2], 1.0);
        Eigen::Vector4d BR_foot_tgt_base_hg = world2rbt * BR_foot_tgt_world;
        vector3_t BR_foot_tgt_base = BR_foot_tgt_base_hg.head(3);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_foot_tgt_world: " << positions[FL].transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_foot_tgt_world: " << positions[FR].transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_foot_tgt_world: " << positions[BL].transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_foot_tgt_world: " << positions[BR].transpose());

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_foot_tgt_base: " << FL_foot_tgt_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_foot_tgt_base: " << FR_foot_tgt_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_foot_tgt_base: " << BL_foot_tgt_base.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_foot_tgt_base: " << BR_foot_tgt_base.transpose());

        // vector3_t FL_calf_posn = interface_->getKinematicModel().calfPositionInOriginFrame(FL, basePose, qJoints);
        // vector3_t FR_calf_posn = interface_->getKinematicModel().calfPositionInOriginFrame(FR, basePose, qJoints);
        // vector3_t BL_calf_posn = interface_->getKinematicModel().calfPositionInOriginFrame(BL, basePose, qJoints);
        // vector3_t BR_calf_posn = interface_->getKinematicModel().calfPositionInOriginFrame(BR, basePose, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_calf_posn: " << FL_calf_posn.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_calf_posn: " << FR_calf_posn.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_calf_posn: " << BL_calf_posn.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_calf_posn: " << BR_calf_posn.transpose());    

        x_world << (FL_foot_posn_world - positions[FL]), 
                    (FR_foot_posn_world - positions[FR]),
                    (BL_foot_posn_world - positions[BL]), 
                    (BR_foot_posn_world - positions[BR]);

        // x << (FL_foot_posn_world - positions[FL]), 
        //      (FR_foot_posn_world - positions[FR]),
        //      (BL_foot_posn_world - positions[BL]), 
        //      (BR_foot_posn_world - positions[BR]);
        x_base << (FL_foot_posn_base - FL_foot_tgt_base), 
                    (FR_foot_posn_base - FR_foot_tgt_base),
                    (BL_foot_posn_base - BL_foot_tgt_base), 
                    (BR_foot_posn_base - BR_foot_tgt_base);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "      IK projection " << projection_iteration << " error (base): " << x_base.norm() << " error (world) " << x_world.norm());

        inContact = x_base.norm() < epsilon;
                    //  (std::abs(x[0]) < epsilon &&
                    //  std::abs(x[1]) < epsilon &&
                    //  std::abs(x[2]) < epsilon &&
                    //  std::abs(x[3]) < epsilon);

        // TODO: check calf position IN BASE FRAME against foot postion

        // vector3_t FL_calf_posn_base_frame = interface_->getKinematicModel().positionBaseToCalfInBaseFrame(FL, qJoints);
        // vector3_t FR_calf_posn_base_frame = interface_->getKinematicModel().positionBaseToCalfInBaseFrame(FR, qJoints);
        // vector3_t BL_calf_posn_base_frame = interface_->getKinematicModel().positionBaseToCalfInBaseFrame(BL, qJoints);
        // vector3_t BR_calf_posn_base_frame = interface_->getKinematicModel().positionBaseToCalfInBaseFrame(BR, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_calf_posn_base_frame: " << FL_calf_posn_base_frame.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_calf_posn_base_frame: " << FR_calf_posn_base_frame.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_calf_posn_base_frame: " << BL_calf_posn_base_frame.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_calf_posn_base_frame: " << BR_calf_posn_base_frame.transpose());    

        // vector3_t FL_foot_posn_world_base_frame = interface_->getKinematicModel().positionBaseToFootInBaseFrame(FL, qJoints);
        // vector3_t FR_foot_posn_world_base_frame = interface_->getKinematicModel().positionBaseToFootInBaseFrame(FR, qJoints);
        // vector3_t BL_foot_posn_world_base_frame = interface_->getKinematicModel().positionBaseToFootInBaseFrame(BL, qJoints);
        // vector3_t BR_foot_posn_world_base_frame = interface_->getKinematicModel().positionBaseToFootInBaseFrame(BR, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FL_foot_posn_world_base_frame: " << FL_foot_posn_world_base_frame.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          FR_foot_posn_world_base_frame: " << FR_foot_posn_world_base_frame.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BL_foot_posn_world_base_frame: " << BL_foot_posn_world_base_frame.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "          BR_foot_posn_world_base_frame: " << BR_foot_posn_world_base_frame.transpose());    

        // collisionFree = true;
        
        // replace with some vector based check, fails on stairs
        // project this onto normal or something instead of just taking Z?
        // if (FL_calf_posn[2] < (FL_foot_posn_world + Z_ANKLE_OFFSET * localContactNormal)[2])
        //     collisionFree = false;
        // if (FR_calf_posn[2] < (FR_foot_posn_world + Z_ANKLE_OFFSET * localContactNormal)[2])
        //     collisionFree = false;
        // if (BL_calf_posn[2] < (BL_foot_posn_world + Z_ANKLE_OFFSET * localContactNormal)[2])
        //     collisionFree = false;    
        // if (BR_calf_posn[2] < (BR_foot_posn_world + Z_ANKLE_OFFSET * localContactNormal)[2])
        //     collisionFree = false;                    
        // RCLCPP_INFO_STREAM(node_->get_logger(), "FL_calf_joint: " << FL_calf_posn.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "FR_calf_joint: " << FR_calf_posn.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "BL_calf_joint: " << BL_calf_posn.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "BR_calf_joint: " << BR_calf_posn.transpose());

        // double min_calf_z_posn = std::min({FL_calf_posn[2], FR_calf_posn[2],
        //                                    BL_calf_posn[2], BR_calf_posn[2]});
                                        
        // RCLCPP_INFO_STREAM(node_->get_logger(), "min_calf_posn: " << min_calf_z_posn);

        // collisionFree = min_calf_z_posn > (// (min_calf_z_posn > (Z_OFFSET + 0.05));

        foundTransition = (inContact); //  && isStable && collisionFree
        if (foundTransition)
            break;

        // calculate Jacobians
        J_FL.setZero(); 
        J_FR.setZero(); 
        J_BL.setZero(); 
        J_BR.setZero();

        J_FL = interface_->getKinematicModel().baseToFootJacobianInBaseFrame(FL, qJoints);
        J_FR = interface_->getKinematicModel().baseToFootJacobianInBaseFrame(FR, qJoints);
        J_BL = interface_->getKinematicModel().baseToFootJacobianInBaseFrame(BL, qJoints);
        J_BR = interface_->getKinematicModel().baseToFootJacobianInBaseFrame(BR, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "J_FL: " << J_FL);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "J_FR: " << J_FR);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "J_BL: " << J_BL);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "J_BR: " << J_BR);

        J_total.setZero();
        J_total.block(0, 0, 3, 12) = J_FL.template bottomRows<3>(); // extract position rows of jacobian
        J_total.block(3, 0, 3, 12) = J_FR.template bottomRows<3>(); // extract position rows of jacobian
        J_total.block(6, 0, 3, 12) = J_BL.template bottomRows<3>(); // extract position rows of jacobian
        J_total.block(9, 0, 3, 12) = J_BR.template bottomRows<3>(); // extract position rows of jacobian

        // RCLCPP_INFO_STREAM(node_->get_logger(), "J_total size: " << J_total.rows() << ", " << J_total.cols());

        // RCLCPP_INFO_STREAM(node_->get_logger(), "J_total" << J_total);

        Eigen::MatrixXd pinv = J_total.completeOrthogonalDecomposition().pseudoInverse();

        // RCLCPP_INFO_STREAM(node_->get_logger(), "pinv size: " << pinv.rows() << ", " << pinv.cols());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "pinv: " << pinv);

        // pinv size: 18 x 12
        double alpha = 1.0; // + 1.0 * (1.0 - double(projection_iteration) / num_projection_iterations); // learning rate
        const joint_coordinate_t new_q_joints = qJoints - alpha * pinv * x_base;

        new_q.tail(12) = new_q_joints; 
    
        // RCLCPP_INFO_STREAM(node_->get_logger(), "  projected q: " << new_q.transpose());

        enforceJointLimits(new_q, contactCenter, localContactNormal);
    
        // RCLCPP_INFO_STREAM(node_->get_logger(), "  projected q (after joint limits): " << new_q.transpose());
    }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  projected q: " << new_q.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  final error: " << x.norm());

    return foundTransition;

}


vector3_t ConstraintManager::getLegPositionConstraint(ModeFamily * mf, 
                                                            const int & legIdx)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      [getLegPositionConstraint]");

    bool foot_offset = true;
    bool inflated_region = true;

    return mf->getRegionCoordinatesWorldFrame(legIdx, foot_offset, inflated_region); 
}

void ConstraintManager::updateLegsPositionConstraints()
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[updateLegsPositionConstraints()]:");

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  current phase: " << graph_->contactSequence->getCurrentMode());

    ModeFamilyNode * currentNodeTo = graph_->getCurrentTransition()->getNodeTo();

    const base_coordinate_t basePose = getBasePose(current_x);
    const joint_coordinate_t qJoints = getJointPositions(current_x);

    for (int leg = 0; leg < 4; leg++)
    {
        // foot_frame_id = model.getBodyId(FootLinks.at(i));
        positions[leg] = (!currentNodeTo->modeFamily->isLegInSwing(leg) && graph_->getLeadCounter() > 0) ? 
                            getLegPositionConstraint(currentNodeTo->modeFamily, leg) : 
                            interface_->getKinematicModel().footPositionInOriginFrame(leg, basePose, qJoints);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "  positions[" << LegNameShort.at(leg) << "]: " << positions[leg].transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "  actual " << LegNameShort.at(leg) << ": " << interface_->getKinematicModel().footPositionInOriginFrame(leg, basePose, qJoints).transpose());
    }
}

void ConstraintManager::updateLegsPositionConstraintsToStart()
{
    for (int i = 0; i < 4; i++)
        positions[i] = getLegPositionConstraint(graph_->getStartModeFamily(), i);
}
                   
int ConstraintManager::attachRegionToLeg(const std::vector<switched_model::ConvexTerrain> & allRegions,
                                            const int & legIdx,
                                            const comkino_state_t & x)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "   [attachRegionToLeg]: ");
    RCLCPP_INFO_STREAM(node_->get_logger(), "          legIdx: " << legIdx);

    int foot_id = -1;

    // vector3_t foot_posn = data.oMf[foot_frame_id].translation();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "          x: " << x.transpose());

    const base_coordinate_t basePose = getBasePose(x);
    const joint_coordinate_t qJoints = getJointPositions(x);

    vector3_t foot_posn = interface_->getKinematicModel().footPositionInOriginFrame(legIdx, basePose, qJoints);

    RCLCPP_INFO_STREAM(node_->get_logger(), "          " << LegName.at(legIdx) << " position: " << foot_posn.transpose());

    double min_diff = 1.0;
    double eps = 0.5; // raised this up to find regions for stairs

    for (int i = 0; i < allRegions.size(); i++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "    Checking region " << i);

        switched_model::ConvexTerrain region = allRegions[i];

        double dist = 1e30;
        dist = (foot_posn - region.plane.positionInWorld).norm();

        scalar_t signedDistanceFromPlane = switched_model::terrainSignedDistanceFromPositionInWorld(foot_posn, region.plane);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "           signedDistanceFromPlane: " << signedDistanceFromPlane);

        bool normalCheck = (std::abs(signedDistanceFromPlane) < 2 * FOOT_RADIUS);
        
        // if (normalCheck)
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "           normalCheck passed");
        // else
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "           normalCheck failed");

        const vector3_t local_p = switched_model::positionInTerrainFrameFromPositionInWorld(foot_posn, region.plane);
        const vector2_t local_2d_p(local_p.x(), local_p.y());

        // RCLCPP_INFO_STREAM(node_->get_logger(), "           local_2d_p: " << local_2d_p.transpose());

        // RCLCPP_INFO_STREAM(node_->get_logger(), "    regions_[legIdx].plane.positionInWorld: " << regions_[legIdx].plane.positionInWorld.transpose()); 
        // RCLCPP_INFO_STREAM(node_->get_logger(), "    regions_[legIdx].boundary: "); 
        // for (int i = 0; i < regions_[legIdx].boundary.size(); i++)
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "      " << regions_[legIdx].boundary[i].transpose());
        
        bool projectedPointInsidePlane = isPointWithinConvex2dPolygonBoundary(region.boundary, local_2d_p);

        // if (projectedPointInsidePlane)
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "           projectedPointInsidePlane passed");
        // else
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "           projectedPointInsidePlane failed");

        if (projectedPointInsidePlane && normalCheck)
        {
            foot_id = i;
            break;
        }

        // if (dist < min_diff)
        // {
        //     min_diff = dist;

        //     if (min_diff < eps)
        //         foot_id = i;
        // }        
    }

    if (foot_id == -1)
        RCLCPP_INFO_STREAM(node_->get_logger(), "          no bar found for " + LegName.at(legIdx) + " foot");

    RCLCPP_INFO_STREAM(node_->get_logger(), "          setting " << LegName.at(legIdx) << " foot to " << foot_id);

    return foot_id;
}

ModeFamilyNodeID ConstraintManager::getStartModeFamilyNodeID(const comkino_state_t & latest_x,
                                                                const comkino_input_t & latest_u,
                                                                const std::vector<switched_model::ConvexTerrain> & allRegions)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[getStartModeFamily]");

    start_x = latest_x;
    start_u = latest_u;

    ModeFamilyNodeID startStanceID = ModeFamilyNodeID();

    for (int l = 0; l < 4; l++)
    {
        int foot_id = attachRegionToLeg(allRegions, l, start_x); // model, data, 

        // RCLCPP_INFO_STREAM(node_->get_logger(), "setting foot " << i << " ID to " << foot_v << ", " << foot_h);

        if (foot_id == -1)
            throw std::runtime_error("Did not find start mode family");
        
        startStanceID.setRegionID(l, foot_id);

    }

    current_x = start_x;
    target_x = start_x;   
    current_u = start_u;
    target_u = start_u; 

    return startStanceID;
}

// ModeFamilyNodeID ConstraintManager::getGoalModeFamilyNodeID(const vector3_t & torsoGoalPosition,
//                                                             const std::vector<switched_model::ConvexTerrain> & allRegions)
// {
//     RCLCPP_INFO_STREAM(node_->get_logger(), "[getGoalModeFamily]");

//     RCLCPP_INFO_STREAM(node_->get_logger(), "      torsoGoalPosition: " << torsoGoalPosition.transpose());

//     std::default_random_engine generator;
//     std::uniform_real_distribution<double> com_xy_distribution(-0.05, 0.05);
    
//     ModeFamilyNodeID goalStanceID = ModeFamilyNodeID();

//     bool invalid = false;

//     for (int i = 0; i < 25; i++)
//     {
//         Eigen::VectorXd temp_x = Eigen::VectorXd::Zero(STATE_DIM);

//         // torso orientation (euler angles) - assuming start and goal are oriented the same
//         temp_x[0] = start_x[0];  // q_x
//         temp_x[1] = start_x[1];  // q_y
//         temp_x[2] = start_x[2];  // q_z

//         // set the torso position
//         temp_x.segment(3, 3) = torsoGoalPosition;
//         if (i > 0)
//         {
//             // perturb the torso position
//             temp_x[3] += com_xy_distribution(generator);
//             temp_x[4] += com_xy_distribution(generator);
//             temp_x[5] += 0.0;
//         }

//         // set rest to default

//         // Leg 0: front left leg 
//         temp_x[12] = 0.0; // hip
//         temp_x[13] = 0.8; // knee
//         temp_x[14] = -1.60; // ankle

//         // Leg 1: front right leg
//         temp_x[15] = 0.0; // hip
//         temp_x[16] = 0.80; // knee
//         temp_x[17] = -1.60; // ankle

//         // Leg 2: back left leg
//         temp_x[18] = 0.0; // hip
//         temp_x[19] = 0.80; // knee
//         temp_x[20] = -1.60; // ankle

//         // Leg 3: back right leg
//         temp_x[21] = 0.0; // hip
//         temp_x[22] = 0.80; // knee
//         temp_x[23] = -1.60; // ankle

//         // RCLCPP_INFO_STREAM(node_->get_logger(), "      temp: " << temp.transpose());

//         invalid = false;
//         for (int l = 0; l < 4; l++)
//         {
//             int foot_id = attachRegionToLeg(allRegions, l, temp_x); // model, data, 
            
//             if (foot_id == -1)
//             {
//                 invalid = true;
//                 break;
//             }

//             goalStanceID.setRegionID(l, foot_id);
    
//         }

//         if (invalid)
//             continue;
//         else
//             break;
//     }

//     if (invalid)
//         throw std::runtime_error("getGoalModeFamily failed");

//     return goalStanceID;
// }

std::string ConstraintManager::descriptionPositions()
{
    std::string str = "[Quadruped Constraints] Leg positions:\n";
    str += "  Leg FL: (" + std::to_string(positions[FL][0]) + ", " + std::to_string(positions[FL][1]) + ", " + std::to_string(positions[FL][2]) + ")\n" +
           "  Leg FR: (" + std::to_string(positions[FR][0]) + ", " + std::to_string(positions[FR][1]) + ", " + std::to_string(positions[FR][2]) + ")\n" +
           "  Leg BL: (" + std::to_string(positions[BL][0]) + ", " + std::to_string(positions[BL][1]) + ", " + std::to_string(positions[BL][2]) + ")\n" +
           "  Leg BR: (" + std::to_string(positions[BR][0]) + ", " + std::to_string(positions[BR][1]) + ", " + std::to_string(positions[BR][2]) + ")\n";

    return str;
}

}  // namespace mmp

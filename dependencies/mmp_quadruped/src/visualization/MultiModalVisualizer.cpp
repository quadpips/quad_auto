#include <pinocchio/fwd.hpp> // NEED TO KEEP THIS AT TOP

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <mmp_quadruped/visualization/MultiModalVisualizer.h>


namespace mmp {

MultiModalVisualizer::MultiModalVisualizer(const rclcpp::Node::SharedPtr &node, 
                                            std::shared_ptr<Go2Visualizer> & go2Visualizer)
{
    node_ = node;
    go2Visualizer_ = go2Visualizer;
    launchVisualizerNode(node);
}

void MultiModalVisualizer::launchVisualizerNode(const rclcpp::Node::SharedPtr &node)
{
    leadPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/lead", 1);
    torsoPathPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/torsoPath", 1);
    superquadricPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/superquadrics", 1);
    
    shrinkingPolygonsPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("/shrinkingPolygons", 1);

    // publishSuperquadrics();

    torsoLocalGoalPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("torsoLocalGoal", 1);
    torsoGlobalGoalPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("torsoGlobalGoal", 1);
    torsoInitPosPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("torsoInitPos", 1);

    return;
}

int sign(const double & input)
{
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else
        return 0;
}

void superquadricHelper(geometry_msgs::msg::Point & p,
                        const vector3_t & sqCenter,
                        const vector3_t & sqDims,
                        const vector3_t & sqCurvature,
                        const double & eta, 
                        const double & w)
{
    p.x = sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
    p.y = sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(std::sin(w)) * std::pow(std::abs(std::sin(w)), 2.0 / sqCurvature[1]);
    p.z = sqDims[2] * sign(std::sin(eta)) * std::pow(std::abs(std::sin(eta)), 2.0 / sqCurvature[2]);
}

void MultiModalVisualizer::visualizeSuperquadrics(const std::vector<Superquadric * > & superquadrics,
                                                    const base_coordinate_t & torso_pose)
{
    // vector3_t p_torso = extractTorsoPosition(torso_pose);
    // vector3_t q_torso = extractTorsoOrientation(torso_pose);

    visualization_msgs::msg::MarkerArray markerArray;
    for (int legIdx = 0; legIdx < 4; legIdx++)
    {
        Superquadric * sq = superquadrics[legIdx];

        vector3_t sqCenter = sq->getCenter();
        vector3_t sqOrientation = sq->getOrientation();
        vector3_t sqDims = sq->getDimension();
        vector3_t sqCurvature = sq->getCurvature();

        Eigen::Quaterniond q_superquadric2rbt = switched_model::quaternionBaseToOrigin(sqOrientation);

        // double roll = q_torso[0] + sqOrientation[0];
        // double pitch = q_torso[1] + sqOrientation[1];
        // double yaw = q_torso[2] + sqOrientation[2];

        // double qx = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        // double qy = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
        // double qz = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
        // double qw = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "                                          yaw: " << yaw );
        // RCLCPP_INFO_STREAM(node_->get_logger(), "                                          pitch: " << pitch );
        // RCLCPP_INFO_STREAM(node_->get_logger(), "                                          roll: " << roll );

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = "superquadric";
        marker.id = legIdx;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = sqCenter[0];
        marker.pose.position.y = sqCenter[1];
        marker.pose.position.z = sqCenter[2];
        marker.pose.orientation.x = q_superquadric2rbt.x();
        marker.pose.orientation.y = q_superquadric2rbt.y();
        marker.pose.orientation.z = q_superquadric2rbt.z();
        marker.pose.orientation.w = q_superquadric2rbt.w();
        marker.scale.x = 1.0; // 0.01;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = getColor(feetColorMap_[legIdx]);
        marker.color.a = 0.8;

        // vector3_t epsilon(0.5, 0.5, 0.5);
        // vector3_t C(0.15, 0.175, 0.15);

        // vector3_t center(1.0, 0.0, 0.0);

        // from: https://en.wikipedia.org/wiki/Superquadrics

        int n = 60;

        Eigen::VectorXd x(n * n);
        Eigen::VectorXd y(n * n);
        Eigen::VectorXd z(n * n);

        double etamax = M_PI/2;
        double etamin = -M_PI/2;
        double wmax = M_PI;
        double wmin = -M_PI;
        double deta = (etamax-etamin)/n;
        double dw = (wmax-wmin)/n;

        for (int i = 0; i <= n; i++)
        {
            for (int j = 0; j <= n; j++)
            {
                geometry_msgs::msg::Point p1;
                double eta = etamin + (i-1) * deta;
                double w   = wmin + (j-1) * dw;            
                superquadricHelper(p1, sqCenter, sqDims, sqCurvature, eta, w);

                geometry_msgs::msg::Point p2;
                eta = etamin + ((i+1)-1) * deta;
                w = wmin + (j-1) * dw;            
                superquadricHelper(p2, sqCenter, sqDims, sqCurvature, eta, w);

                geometry_msgs::msg::Point p3;
                eta = etamin + (i-1) * deta;
                w = wmin + ((j+1)-1) * dw;            
                superquadricHelper(p3, sqCenter, sqDims, sqCurvature, eta, w);

                geometry_msgs::msg::Point p4;
                eta = etamin + ((i+1)-1) * deta;
                w = wmin + ((j+1)-1) * dw;            
                superquadricHelper(p4, sqCenter, sqDims, sqCurvature, eta, w);

                marker.points.push_back(p3);
                marker.points.push_back(p2);
                marker.points.push_back(p1); 

                marker.points.push_back(p2);
                marker.points.push_back(p3);
                marker.points.push_back(p4);             
            }
        }

        geometry_msgs::msg::Point p1;
        double eta = etamin + n * deta;
        double w   = wmin + n * dw;            
        superquadricHelper(p1, sqCenter, sqDims, sqCurvature, eta, w);

        geometry_msgs::msg::Point p2;
        eta = etamin + (0.0) * deta;
        w = wmin + n * dw;            
        superquadricHelper(p2, sqCenter, sqDims, sqCurvature, eta, w);

        geometry_msgs::msg::Point p3;
        eta = etamin + n * deta;
        w = wmin + (0.0) * dw;            
        superquadricHelper(p3, sqCenter, sqDims, sqCurvature, eta, w);

        geometry_msgs::msg::Point p4;
        eta = etamin + (0.0) * deta;
        w = wmin + (0.0) * dw;            
        superquadricHelper(p4, sqCenter, sqDims, sqCurvature, eta, w);

        marker.points.push_back(p3);
        marker.points.push_back(p2);
        marker.points.push_back(p1); 

        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p4);     

        markerArray.markers.push_back(marker);
    }

    superquadricPublisher_->publish(markerArray);
}

void MultiModalVisualizer::publishTorsoLocalWaypoint(const base_coordinate_t & torsoLocalWaypointPose,
                                                    const double & localGoalRegionTolerance)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "publishTorsoGlobalGoal()" );
    visualization_msgs::msg::Marker torsoLocalGoalMarker;

    vector3_t torsoLocalGoalPosition = extractTorsoPosition(torsoLocalWaypointPose);

    torsoLocalGoalMarker.header.frame_id = "odom";
    torsoLocalGoalMarker.header.stamp = node_->get_clock()->now();
    torsoLocalGoalMarker.lifetime = rclcpp::Duration::from_seconds(0); // meant to be forever
    torsoLocalGoalMarker.id = 0;
    torsoLocalGoalMarker.type = visualization_msgs::msg::Marker::SPHERE;
    torsoLocalGoalMarker.action = visualization_msgs::msg::Marker::ADD;
    torsoLocalGoalMarker.pose.position.x = torsoLocalGoalPosition[0]; // + position_dist(generator);
    torsoLocalGoalMarker.pose.position.y = torsoLocalGoalPosition[1]; // + position_dist(generator);
    torsoLocalGoalMarker.pose.position.z = torsoLocalGoalPosition[2];
    torsoLocalGoalMarker.pose.orientation.x = 0.0;
    torsoLocalGoalMarker.pose.orientation.y = 0.0;
    torsoLocalGoalMarker.pose.orientation.z = 0.0;
    torsoLocalGoalMarker.pose.orientation.w = 1.0;
    torsoLocalGoalMarker.scale.x = localGoalRegionTolerance;
    torsoLocalGoalMarker.scale.y = localGoalRegionTolerance;
    torsoLocalGoalMarker.scale.z = localGoalRegionTolerance;

    torsoLocalGoalMarker.color.a = 0.5;
    torsoLocalGoalMarker.color.r = 0.0;
    torsoLocalGoalMarker.color.g = 1.0;
    torsoLocalGoalMarker.color.b = 0.5;

    torsoLocalGoalPublisher_->publish(torsoLocalGoalMarker);
}


void MultiModalVisualizer::publishTorsoInitPos(const base_coordinate_t & torsoInitPos,
                                                    const double & globalGoalRegionTolerance)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "publishTorsoInitPos()" );
    visualization_msgs::msg::Marker torsoInitPosMarker;

    vector3_t torsoInitPosition = extractTorsoPosition(torsoInitPos);

    torsoInitPosMarker.header.frame_id = "odom";
    torsoInitPosMarker.header.stamp = node_->get_clock()->now();
    torsoInitPosMarker.lifetime = rclcpp::Duration::from_seconds(0); // meant to be forever
    torsoInitPosMarker.id = 0;
    torsoInitPosMarker.type = visualization_msgs::msg::Marker::SPHERE;
    torsoInitPosMarker.action = visualization_msgs::msg::Marker::ADD;
    torsoInitPosMarker.pose.position.x = torsoInitPosition[0]; // + position_dist(generator);
    torsoInitPosMarker.pose.position.y = torsoInitPosition[1]; // + position_dist(generator);
    torsoInitPosMarker.pose.position.z = torsoInitPosition[2];
    torsoInitPosMarker.pose.orientation.x = 0.0;
    torsoInitPosMarker.pose.orientation.y = 0.0;
    torsoInitPosMarker.pose.orientation.z = 0.0;
    torsoInitPosMarker.pose.orientation.w = 1.0;
    torsoInitPosMarker.scale.x = globalGoalRegionTolerance;
    torsoInitPosMarker.scale.y = globalGoalRegionTolerance;
    torsoInitPosMarker.scale.z = globalGoalRegionTolerance;

    torsoInitPosMarker.color.a = 0.5;
    torsoInitPosMarker.color.r = 0.0;
    torsoInitPosMarker.color.g = 1.0;
    torsoInitPosMarker.color.b = 0.0;

    torsoInitPosPublisher_->publish(torsoInitPosMarker);
}

void MultiModalVisualizer::publishTorsoGlobalGoal(const base_coordinate_t & torsoGlobalGoalPose,
                                                    const double & globalGoalRegionTolerance)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "publishTorsoGlobalGoal()" );
    visualization_msgs::msg::Marker torsoGlobalGoalMarker;

    vector3_t torsoGlobalGoalPosition = extractTorsoPosition(torsoGlobalGoalPose);

    torsoGlobalGoalMarker.header.frame_id = "odom";
    torsoGlobalGoalMarker.header.stamp = node_->get_clock()->now();
    torsoGlobalGoalMarker.lifetime = rclcpp::Duration::from_seconds(0); // meant to be forever
    torsoGlobalGoalMarker.id = 0;
    torsoGlobalGoalMarker.type = visualization_msgs::msg::Marker::SPHERE;
    torsoGlobalGoalMarker.action = visualization_msgs::msg::Marker::ADD;
    torsoGlobalGoalMarker.pose.position.x = torsoGlobalGoalPosition[0]; // + position_dist(generator);
    torsoGlobalGoalMarker.pose.position.y = torsoGlobalGoalPosition[1]; // + position_dist(generator);
    torsoGlobalGoalMarker.pose.position.z = torsoGlobalGoalPosition[2];
    torsoGlobalGoalMarker.pose.orientation.x = 0.0;
    torsoGlobalGoalMarker.pose.orientation.y = 0.0;
    torsoGlobalGoalMarker.pose.orientation.z = 0.0;
    torsoGlobalGoalMarker.pose.orientation.w = 1.0;
    torsoGlobalGoalMarker.scale.x = globalGoalRegionTolerance;
    torsoGlobalGoalMarker.scale.y = globalGoalRegionTolerance;
    torsoGlobalGoalMarker.scale.z = globalGoalRegionTolerance;

    torsoGlobalGoalMarker.color.a = 0.5;
    torsoGlobalGoalMarker.color.r = 1.0;
    torsoGlobalGoalMarker.color.g = 0.0;
    torsoGlobalGoalMarker.color.b = 0.0;

    torsoGlobalGoalPublisher_->publish(torsoGlobalGoalMarker);
}

void MultiModalVisualizer::publishTorsoPath(std::vector<TorsoStateNode::TorsoTransition *> torsoPath)
{
    // clear markers
    visualization_msgs::msg::MarkerArray clear_arr;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.id = 0;
    clear_marker.ns = "clear";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_arr.markers.push_back(clear_marker);
    torsoPathPublisher_->publish(clear_arr);

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker marker;

    double counter = 0.0;
    for (TorsoStateNode::TorsoTransition * trans : torsoPath)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = "torso_path";
        marker.id = counter;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "  marker.id: " << marker.id );

        base_coordinate_t torsoPose = trans->getNodeFrom()->getPoseWorldFrame();
        vector3_t torsoPosition = extractTorsoPosition(torsoPose);
        vector3_t torsoOrientation = extractTorsoOrientation(torsoPose);
        marker.pose.position.x = torsoPosition[0];
        marker.pose.position.y = torsoPosition[1];
        marker.pose.position.z = torsoPosition[2];
        marker.pose.orientation.x = 0.0; // TODO
        marker.pose.orientation.y = 0.0; // TODO
        marker.pose.orientation.z = 0.0; // TODO
        marker.pose.orientation.w = 1.0; // TODO

        // RCLCPP_INFO_STREAM(node_->get_logger(), "  marker.pose: " << marker.pose );

        marker.scale.x = 0.30; 
        marker.scale.y = 0.30;
        marker.scale.z = 0.20;
        marker.color.a = 1.0;
        marker.color.r = (counter / torsoPath.size());
        marker.color.g = 1.0 - (counter / torsoPath.size());
        marker.color.b = 0.0;

        counter = counter + 1;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "  pre-add" );
        markerArray.markers.push_back(marker); // [marker.id] = 
    }
    // visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "torso_path";
    marker.id = counter;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  marker.id: " << marker.id );

    base_coordinate_t torsoPose = torsoPath[torsoPath.size() - 1]->getNodeTo()->getPoseWorldFrame();
    vector3_t torsoPosition = extractTorsoPosition(torsoPose);
    vector3_t torsoOrientation = extractTorsoOrientation(torsoPose);
    marker.pose.position.x = torsoPosition[0];
    marker.pose.position.y = torsoPosition[1];
    marker.pose.position.z = torsoPosition[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  marker.pose: " << marker.pose );

    marker.scale.x = 0.30; 
    marker.scale.y = 0.30;
    marker.scale.z = 0.20;
    marker.color.a = 1.0;
    marker.color.r = (counter / torsoPath.size());
    marker.color.g = 1.0 - (counter / torsoPath.size());
    marker.color.b = 0.0;
    // RCLCPP_INFO_STREAM(node_->get_logger(), "  pre-add" );
    markerArray.markers.push_back(marker); // [marker.id] = 

    torsoPathPublisher_->publish(markerArray);
    return;
}

void MultiModalVisualizer::publishLead(std::vector<ModeFamilyNode::Transition *> & lead)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "publishLead()" );

    // clear markers
    visualization_msgs::msg::MarkerArray clear_arr;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.id = 0;
    clear_marker.ns = "clear";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_arr.markers.push_back(clear_marker);
    leadPublisher_->publish(clear_arr);

    visualization_msgs::msg::MarkerArray markerArray;
    // markerArray.markers.reserve(5 * lead.size());

    // bool foot_offset = true;
    // bool inflated_region = true;

    double counter = 0.0;
    for (ModeFamilyNode::Transition * trans : lead)
    {
        std::vector<vector3_t> footPositions;
        for (int leg_idx = 0; leg_idx < 4; leg_idx++)
        {
            ModeFamily * mf = (trans->getNodeFrom()->modeFamily->isLegInSwing(leg_idx)) ?
                                trans->getNodeTo()->modeFamily : 
                                trans->getNodeFrom()->modeFamily;

            vector3_t footPosition = mf->getRegionCoordinatesWorldFrame(leg_idx);
            
            footPositions.push_back(footPosition);

            // RCLCPP_INFO_STREAM(node_->get_logger(), "  footPosition: " << footPosition.transpose() );

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; // "base_aligned";
            marker.header.stamp = node_->get_clock()->now();
            marker.ns = "lead";
            marker.id = counter++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // RCLCPP_INFO_STREAM(node_->get_logger(), "  marker.id: " << marker.id );

            marker.pose.position.x = footPosition[0];
            marker.pose.position.y = footPosition[1];
            marker.pose.position.z = footPosition[2];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // RCLCPP_INFO_STREAM(node_->get_logger(), "  marker.pose: " << marker.pose );

            marker.scale.x = 0.025; 
            marker.scale.y = 0.025;
            marker.scale.z = 0.025;
            marker.color = getColor(feetColorMap_[leg_idx]);
            marker.color.a = 1.0 - (counter / (5.0 * lead.size()));
            // marker.color.r = 0.0;
            // marker.color.g = 1.0;
            // marker.color.b = 0.0;
            // RCLCPP_INFO_STREAM(node_->get_logger(), "  pre-add" );
            markerArray.markers.push_back(marker); // [marker.id] = 
            // RCLCPP_INFO_STREAM(node_->get_logger(), "  post-add" );
        }

    }
    
    leadPublisher_->publish(markerArray);
}

void MultiModalVisualizer::visualizeFailedStep(const vector_array_t & failed_step_state_trajectory,
                                               const vector_array_t & failed_step_input_trajectory,
                                               const scalar_array_t & failed_step_time_trajectory)
{
    double real_time_factor = 0.25;
    size_t num_path_visualizations = 5;

    visualizePathNTimes(failed_step_state_trajectory, failed_step_input_trajectory, failed_step_time_trajectory,
                        real_time_factor);
}

void MultiModalVisualizer::visualizeReferenceTrajectory(const vector_array_t & state_trajectory, 
                                                        const vector_array_t & input_trajectory,
                                                        const scalar_array_t & time_trajectory,
                                                        const double & real_time_factor,
                                                        const double & num_path_visualizations)
{
    if (state_trajectory.size() == 0)
        return;

    for (size_t path_vis = 0; path_vis < num_path_visualizations; path_vis++) 
    {
        for (size_t indx = 0; indx < (state_trajectory.size() - 1); indx++) 
        {
            ocs2::SystemObservation sol;
            sol.state = state_trajectory[indx];
            sol.input = input_trajectory[indx];
            const auto timeStamp = node_->get_clock()->now();

            double currTime = time_trajectory[indx];
            double nextTime = time_trajectory[indx+1];

            go2Visualizer_->publishObservation(timeStamp, sol);
            rclcpp::Rate(real_time_factor * 1.0 / (nextTime - currTime)).sleep();
        }
    }
}


/*
quadruped::EndEffectorLinearConstraint::Config MultiModalVisualizer::populateConfig(const double & time,
                                                                                    const vector3_t & bot_left_pt_,
                                                                                    const vector3_t & bot_right_pt_,
                                                                                    const vector3_t & top_left_pt_,
                                                                                    const vector3_t & top_right_pt_,
                                                                                    const vector3_t & mid_pt_,
                                                                                    const double & slack_factor)
{
    quadruped::EndEffectorLinearConstraint::Config config;

    // s is a slack variable
    scalar_t swingTime = 0.40;
    scalar_t maxSwingTime = 0.45;
    scalar_t swingTimeLeft = maxSwingTime - time;
    scalar_t s = (swingTimeLeft / swingTime) * slack_factor;

    config.Ax.setZero(4, 3);
    config.Av.setZero(4, 3);
    config.b.setZero(4);

    double x_coeff, y_coeff, offset;
    double x_diff, y_diff;
    double m, b; // slope and y-intercept

    // if (slack_factor == 0.0)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "        swingTimeLeft: " << swingTimeLeft );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "        swingTime: " << swingTime );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "        slack_factor: " << slack_factor );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "        s: " << s );

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_left_pt_: " << bot_left_pt_.transpose() );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_right_pt_: " << bot_right_pt_.transpose() );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         top_left_pt_: " << top_left_pt_.transpose() );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         top_right_pt_: " << top_right_pt_.transpose() );
    // }

    vector3_t mid_to_bot_left_pt = (bot_left_pt_ - mid_pt_);
    // vector3_t mid_to_bot_left_norm = mid_to_bot_left_pt / mid_to_bot_left_pt.norm(); 
    inf_bot_left_pt_ = mid_pt_ + (s + 1) * mid_to_bot_left_pt;

    vector3_t mid_to_bot_right_pt = (bot_right_pt_ - mid_pt_);
    // vector3_t mid_to_bot_right_norm = mid_to_bot_right_pt / mid_to_bot_right_pt.norm();
    inf_bot_right_pt_ = mid_pt_ + (s + 1) * mid_to_bot_right_pt;

    vector3_t mid_to_top_left_pt = (top_left_pt_ - mid_pt_);
    // vector3_t mid_to_top_left_norm = mid_to_top_left_pt / mid_to_top_left_pt.norm();
    inf_top_left_pt_ = mid_pt_ + (s + 1) * mid_to_top_left_pt;

    vector3_t mid_to_top_right_pt = (top_right_pt_ - mid_pt_);
    // vector3_t mid_to_top_right_norm = mid_to_top_right_pt / mid_to_top_right_pt.norm();
    inf_top_right_pt_ = mid_pt_ + (s + 1) * mid_to_top_right_pt;

    vector3_t ptA, ptB;

    double eps = 0.0001;
    // projecting down, only dealing with x/y dimensions
    // loop through four sides of steppable region
    for (int i = 0; i < 4; i++)
    {
        // ss << "         i: " << i << "\n";
        if (i == 0) // bot left to bot right
        {
            ptA = inf_bot_left_pt_; // bot_left_pt_; //
            ptB =  inf_bot_right_pt_; // bot_right_pt_; //
        } else if (i == 1) // bot right to top right
        {
            ptA = inf_bot_right_pt_; // bot_right_pt_; // 
            ptB = inf_top_right_pt_; // top_right_pt_; // 
        } else if (i == 2) // top right to top left
        {
            ptA = inf_top_right_pt_; // top_right_pt_; // 
            ptB = inf_top_left_pt_; // top_left_pt_; //            
        } else // top left to bot left
        {
            ptA = inf_top_left_pt_; // top_left_pt_; // 
            ptB = inf_bot_left_pt_; // bot_left_pt_; //                                
        }

        // ss << "             ptA: " << ptA.transpose() << "\n";
        // ss << "             ptB: " << ptB.transpose() << "\n";

        x_diff = ptB[0] - ptA[0];
        y_diff = ptB[1] - ptA[1];

        // ss << "             x_diff: " << x_diff << "\n";
        // ss << "             y_diff: " << y_diff << "\n";

        if (std::abs(x_diff) <= eps) // checking for vertical lines where m = infinity
        {
            if (y_diff > 0.0)
            {
                x_coeff = -1.0;
                y_coeff = 0.0;
                offset = ptA[0]; // set to x val          
            } else
            {
                x_coeff = 1.0;
                y_coeff = 0.0;
                offset = -ptA[0]; // set to x val       
            }  
        } else
        {
            m = y_diff / x_diff;
            b = ptA[1] - m * ptA[0];    
            if (x_diff > 0.0)
            {
                x_coeff = -m;
                y_coeff = 1.0;
                offset = -b;
            } else 
            {
                x_coeff = m;
                y_coeff = -1.0;
                offset = b;
            }
        }

        // ss << "             x_coeff: " << x_coeff << "\n";
        // ss << "             y_coeff: " << y_coeff << "\n";
        // ss << "             offset: " << offset << "\n";

        config.Ax(i, 0) = x_coeff;
        config.Ax(i, 1) = y_coeff;
        config.b(i) = offset;
    }

    // if (slack_factor == 0.0)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         inf_bot_left_pt_: " << inf_bot_left_pt_.transpose() );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         inf_bot_right_pt_: " << inf_bot_right_pt_.transpose() );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         inf_top_left_pt_: " << inf_top_left_pt_.transpose() );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "         inf_top_right_pt_: " << inf_top_right_pt_.transpose() );
    
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "     config.Ax: " << config.Ax(0, 0) << ", " << config.Ax(0, 1) << ", " << config.Ax(0, 2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(1, 0) << ", " << config.Ax(1, 1) << ", " << config.Ax(1, 2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(2, 0) << ", " << config.Ax(2, 1) << ", " << config.Ax(2, 2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(3, 0) << ", " << config.Ax(3, 1) << ", " << config.Ax(3, 2) );

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "     config.Av: " << config.Av(0, 0) << ", " << config.Av(0, 1) << ", " << config.Av(0, 2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Av(1, 0) << ", " << config.Av(1, 1) << ", " << config.Av(1, 2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Av(2, 0) << ", " << config.Av(2, 1) << ", " << config.Av(2, 2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Av(3, 0) << ", " << config.Av(3, 1) << ", " << config.Av(3, 2) );

    //     RCLCPP_INFO_STREAM(node_->get_logger(), "     config.b: " << config.b(0) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               " << config.b(1) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               " << config.b(2) );
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               " << config.b(3) );    
    // }

    // RCLCPP_INFO_STREAM(node_->get_logger(), ss.str() );

    return config;
}

void MultiModalVisualizer::visualizeShrinkingPolygons(const std::vector<ModeFamilyNode::Transition *> & lead,
                                                        const int & step_counter,
                                                        const double & currTime)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "      [visualizeShrinkingPolygons]" );

    // RCLCPP_INFO_STREAM(node_->get_logger(), "              step_counter: " << step_counter );

    // RCLCPP_INFO_STREAM(node_->get_logger(), "              currTime: " << currTime );

    ModeFamilyNode::Transition * currentTransition = lead.at(step_counter);

    bool foot_offset = true;
    bool inflated_region = true;

    // std::vector<double> zeroVector(2, 0.0);
    // std::vector<double> oneVector(2, 1.0);
    std::vector<double> oneZeroVector = {1.0, 0.0};

    // if (currentTransition->getNodeFrom()->modeFamily->isLegInSwing(FR))
    // {
    //     FR_bot_left = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(FR, zeroVector);
    //     FR_bot_right = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(FR, oneZeroVector);
    //     FR_top_right = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(FR, oneVector);
    // } else
    // {
    //     FR_bot_left = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(FR, zeroVector);
    //     FR_bot_right = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(FR, oneZeroVector);
    //     FR_top_right = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(FR, oneVector);
    // }

    // if (currentTransition->getNodeFrom()->modeFamily->isLegInSwing(BL))
    // {
    //     BL_bot_left = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(BL, zeroVector);
    //     BL_bot_right = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(BL, oneZeroVector);
    //     BL_top_right = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(BL, oneVector);
    // } else
    // {
    //     BL_bot_left = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(BL, zeroVector);
    //     BL_bot_right = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(BL, oneZeroVector);
    //     BL_top_right = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(BL, oneVector);
    // }

    // if (currentTransition->getNodeFrom()->modeFamily->isLegInSwing(BR))
    // {
    //     BR_bot_left = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(BR, zeroVector);
    //     BR_bot_right = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(BR, oneZeroVector);
    //     BR_top_right = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(BR, oneVector);
    // } else
    // {
    //     BR_bot_left = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(BR, zeroVector);
    //     BR_bot_right = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(BR, oneZeroVector);
    //     BR_top_right = currentTransition->getNodeFrom()->modeFamily->getRegionCoordinatesWorldFrame(BR, oneVector);
    // }

    vector3_t bot_left_pt_, bot_right_pt_, top_right_pt_, top_left_pt_, mid_pt_;

    for (int i = 0; i < 4; i++)
    {
        if (currentTransition->getNodeFrom()->modeFamily->isLegInSwing(i))
        {
            bot_left_pt_ = currentTransition->getNodeTo()->modeFamily->getMinPositionBaseFrame(i, foot_offset, inflated_region);
            bot_right_pt_ = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(i, oneZeroVector, foot_offset, inflated_region);
            top_right_pt_ = currentTransition->getNodeTo()->modeFamily->getMaxPositionBaseFrame(i, foot_offset, inflated_region);

            top_left_pt_ = bot_left_pt_ + (top_right_pt_ - bot_right_pt_);

            // RCLCPP_INFO_STREAM(node_->get_logger(), "         for foot: " << LegNameShort.at(i) );

            // RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_left_pt: " << bot_left_pt_.transpose() );
            // RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_right_pt_: " << bot_right_pt_.transpose() );
            // RCLCPP_INFO_STREAM(node_->get_logger(), "         top_left_pt_: " << top_left_pt_.transpose() );
            // RCLCPP_INFO_STREAM(node_->get_logger(), "         top_right_pt_: " << top_right_pt_.transpose() );

            mid_pt_ = (bot_left_pt_ + top_right_pt_) / 2.0;

            // set inflated points
            quadruped::EndEffectorLinearConstraint::Config config;
            if (i == 0)
                config = populateConfig(currTime,
                                        bot_left_pt_,
                                        bot_right_pt_,
                                        top_left_pt_,
                                        top_right_pt_,
                                        mid_pt_,
                                        FL_slack_factor);
            else if (i == 1)
                config = populateConfig(currTime,
                                        bot_left_pt_,
                                        bot_right_pt_,
                                        top_left_pt_,
                                        top_right_pt_,
                                        mid_pt_,
                                        FR_slack_factor);
            else if (i == 2)
                config = populateConfig(currTime,
                                        bot_left_pt_,
                                        bot_right_pt_,
                                        top_left_pt_,
                                        top_right_pt_,
                                        mid_pt_,
                                        BL_slack_factor);
            else if (i == 3)
                config = populateConfig(currTime,
                                        bot_left_pt_,
                                        bot_right_pt_,
                                        top_left_pt_,
                                        top_right_pt_,
                                        mid_pt_,
                                        BR_slack_factor);

            // visualize inflated points
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_aligned";
            marker.header.stamp = ros::Time();
            marker.ns = "inflated_polygon";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.02;
            // marker.scale.y = 0.1;
            // marker.scale.z = 0.1;
            // marker.color.a = 1.0; // Don't forget to set the alpha!
            // marker.color.r = 0.0;
            // marker.color.g = 1.0;
            // marker.color.b = 0.0;
            marker.color = getColor(feetColorMap_[i]);


            geometry_msgs::msg::Point p;
            p.x = inf_bot_left_pt_[0]; p.y = inf_bot_left_pt_[1]; p.z = inf_bot_left_pt_[2];
            marker.points.push_back(p);
            p.x = inf_bot_right_pt_[0]; p.y = inf_bot_right_pt_[1]; p.z = inf_bot_right_pt_[2];
            marker.points.push_back(p);
            p.x = inf_top_right_pt_[0]; p.y = inf_top_right_pt_[1]; p.z = inf_top_right_pt_[2];
            marker.points.push_back(p);
            p.x = inf_top_left_pt_[0]; p.y = inf_top_left_pt_[1]; p.z = inf_top_left_pt_[2];
            marker.points.push_back(p);
            p.x = inf_bot_left_pt_[0]; p.y = inf_bot_left_pt_[1]; p.z = inf_bot_left_pt_[2];
            marker.points.push_back(p);

            shrinkingPolygonsPublisher_.publish(marker);
        } else
        {
            continue;
        }
    }
}

void MultiModalVisualizer::visualizePathWithPolygonsNTimes(MmpInterface & interface,
                                                            const vector_array_t & state_trajectory, 
                                                            const vector_array_t & input_trajectory,
                                                            const scalar_array_t & time_trajectory,
                                                            const double & timeHorizon,         
                                                            const std::vector<ModeFamilyNode::Transition *> & lead,                                                            
                                                            double real_time_factor)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[visualizePathWithPolygons]" );

    if (state_trajectory.size() == 0)
        return;

    double eps = 0.0001;

    const auto& model = interface.getPinocchioInterface().getModel();
    auto& data = interface.getPinocchioInterface().getData();

    vector3_t bot_left_pt_, bot_right_pt_, top_right_pt_, top_left_pt_, mid_pt_;

    bool foot_offset = true;
    bool inflated_region = true;

    // std::vector<double> zeroVector(2, 0.0);
    // std::vector<double> oneVector(2, 1.0);
    std::vector<double> oneZeroVector = {1.0, 0.0};

    std::string input;

    RCLCPP_INFO_STREAM(node_->get_logger(), "Press G to start visualizing..." );
    std::cin >> input;

    while (input == "g")
    {
        int step_counter = 0; // start from 0 because the 0-1 transition is a discrete mode change, nothing there to visualize
        vector_t initial_state;

        for (size_t indx = 0; indx < (state_trajectory.size()-1); indx++) 
        {
            SystemObservation sol;
            sol.state = state_trajectory[indx];
            sol.input = input_trajectory[indx];
            double currTime = time_trajectory[indx];
            double nextTime = time_trajectory[indx+1];         

            // RCLCPP_INFO_STREAM(node_->get_logger(), "  currTime:" << currTime );

            if (std::abs(currTime - 0.05) < eps && std::abs(nextTime - 0.05) < eps) // jump time step
            {
                step_counter += 1;
                initial_state = state_trajectory[indx];
                // RCLCPP_INFO_STREAM(node_->get_logger(), "  step_counter: " << step_counter );

                Eigen::VectorXd initial_q = initial_state.tail<18>();

                // run FK on new configuration
                pinocchio::forwardKinematics(model, data, initial_q);
                pinocchio::computeJointJacobians(model, data);        
                pinocchio::updateFramePlacements(model, data);

                ModeFamilyNode::Transition * currentTransition = lead.at(step_counter);

                // reset values
                FL_slack_factor = 0.0;
                FR_slack_factor = 0.0;
                BL_slack_factor = 0.0;
                BR_slack_factor = 0.0;

                // set the slack factors for the four legs
                for (int leg_idx = 0; leg_idx < 4; leg_idx++)
                {
                    if (currentTransition->getNodeFrom()->modeFamily->isLegInSwing(leg_idx))
                    {
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "          leg_idx: " << leg_idx );

                        bot_left_pt_ = currentTransition->getNodeTo()->modeFamily->getMinPositionBaseFrame(leg_idx, foot_offset, inflated_region);
                        bot_right_pt_ = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(leg_idx, oneZeroVector, foot_offset, inflated_region);
                        top_right_pt_ = currentTransition->getNodeTo()->modeFamily->getMaxPositionBaseFrame(leg_idx, foot_offset, inflated_region);

                        top_left_pt_ = bot_left_pt_ + (top_right_pt_ - bot_right_pt_);

                        mid_pt_ = (bot_left_pt_ + top_right_pt_) / 2.0;

                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_left_pt: " << bot_left_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_right_pt_: " << bot_right_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         top_left_pt_: " << top_left_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         top_right_pt_: " << top_right_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         mid_pt_: " << mid_pt_.transpose() );   

                        vector3_t initial_foothold = data.oMf[model.getBodyId(FootLinks.at(leg_idx))].translation();

                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         initial_foothold: " << initial_foothold.transpose() );

                        // set slack_factor

                        int iter = 0;
                        int maxIters = 1000;

                        bool constraintsNotMet = true;
                        while (constraintsNotMet && iter < maxIters)
                        {
                            constraintsNotMet = false;

                            quadruped::EndEffectorLinearConstraint::Config config;
                            if (leg_idx == 0)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        FL_slack_factor);
                            else if (leg_idx == 1)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        FR_slack_factor);
                            else if (leg_idx == 2)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        BL_slack_factor);
                            else if (leg_idx == 3)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        BR_slack_factor);
                            
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "     config.Ax: " << config.Ax(0, 0) << ", " << config.Ax(0, 1) << ", " << config.Ax(0, 2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(1, 0) << ", " << config.Ax(1, 1) << ", " << config.Ax(1, 2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(2, 0) << ", " << config.Ax(2, 1) << ", " << config.Ax(2, 2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(3, 0) << ", " << config.Ax(3, 1) << ", " << config.Ax(3, 2) << "\n";

                            // RCLCPP_INFO_STREAM(node_->get_logger(), "     config.b : " << config.b(0) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.b(1) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.b(2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.b(3) << "\n";

                            vector_t initialValue = config.Ax * initial_foothold + config.b;

                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "    initialValue: " << initialValue[0] << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "                  " << initialValue[1] << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "                  " << initialValue[2] << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "                  " << initialValue[3] << "\n";

                            for (int i = 0; i < 4; i++)
                            {
                                if (initialValue[i] < 0.0)
                                {
                                    constraintsNotMet = true;
                                    
                                    if (leg_idx == 0)
                                        FL_slack_factor += 0.1;
                                    else if (leg_idx == 1)
                                        FR_slack_factor += 0.1;
                                    else if (leg_idx == 2)
                                        BL_slack_factor += 0.1;
                                    else if (leg_idx == 3)
                                        BR_slack_factor += 0.1;                                                                                                                        

                                    break;
                                }
                            }
                            iter++;
                        } 

                        if (leg_idx == 0)
                        {
                            FL_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial FL_slack_factor: " << FL_slack_factor );
                        } else if (leg_idx == 1)
                        {
                            FR_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial FR_slack_factor: " << FR_slack_factor );
                        } else if (leg_idx == 2)
                        {
                            BL_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial BL_slack_factor: " << BL_slack_factor );
                        } else if (leg_idx == 3)
                        {
                            BR_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial BR_slack_factor: " << BR_slack_factor );
                        }
                    }
                }       
            } else if (currTime >= 0.05 && currTime < 0.45) // swing time
            {
                // build lines
                visualizeShrinkingPolygons(lead, step_counter, currTime);
            }

            const auto timeStamp = ros::Time::now();

            go2Visualizer_->publishObservation(timeStamp, sol);
            ros::Rate(real_time_factor * 1.0 / (nextTime - currTime)).sleep();      
            
            // if (indx == 0)
            //     ros::Duration(5.0).sleep();
        }

        RCLCPP_INFO_STREAM(node_->get_logger(), "Press G to continue visualizing, press another key to stop." );
        std::cin >> input;        
    }
}

void MultiModalVisualizer::visualizePathWithPolygons(MmpInterface & interface,
                                                        const vector_array_t & state_trajectory, 
                                                        const vector_array_t & input_trajectory,
                                                        const scalar_array_t & time_trajectory,
                                                        const double & timeHorizon,         
                                                        const std::vector<ModeFamilyNode::Transition *> & lead,                                                            
                                                        double real_time_factor,
                                                        double num_path_visualizations)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[visualizePathWithPolygons]" );

    if (state_trajectory.size() == 0)
        return;

    double eps = 0.0001;

    const auto& model = interface.getPinocchioInterface().getModel();
    auto& data = interface.getPinocchioInterface().getData();

    vector3_t bot_left_pt_, bot_right_pt_, top_right_pt_, top_left_pt_, mid_pt_;

    bool foot_offset = true;
    bool inflated_region = true;

    // std::vector<double> zeroVector(2, 0.0);
    // std::vector<double> oneVector(2, 1.0);
    std::vector<double> oneZeroVector = {1.0, 0.0};

    for (size_t path_vis = 0; path_vis < num_path_visualizations; path_vis++) 
    {
        int step_counter = 0; // start from 0 because the 0-1 transition is a discrete mode change, nothing there to visualize
        vector_t initial_state;

        for (size_t indx = 0; indx < (state_trajectory.size()-1); indx++) 
        {
            SystemObservation sol;
            sol.state = state_trajectory[indx];
            sol.input = input_trajectory[indx];
            double currTime = time_trajectory[indx];
            double nextTime = time_trajectory[indx+1];         

            // RCLCPP_INFO_STREAM(node_->get_logger(), "  currTime:" << currTime );

            if (std::abs(currTime - 0.05) < eps && std::abs(nextTime - 0.05) < eps) // jump time step
            {
                step_counter += 1;
                initial_state = state_trajectory[indx];
                // RCLCPP_INFO_STREAM(node_->get_logger(), "  step_counter: " << step_counter );

                Eigen::VectorXd initial_q = initial_state.tail<18>();

                // run FK on new configuration
                pinocchio::forwardKinematics(model, data, initial_q);
                pinocchio::computeJointJacobians(model, data);        
                pinocchio::updateFramePlacements(model, data);

                ModeFamilyNode::Transition * currentTransition = lead.at(step_counter);

                // reset values
                FL_slack_factor = 0.0;
                FR_slack_factor = 0.0;
                BL_slack_factor = 0.0;
                BR_slack_factor = 0.0;

                // set the slack factors for the four legs
                for (int leg_idx = 0; leg_idx < 4; leg_idx++)
                {
                    if (currentTransition->getNodeFrom()->modeFamily->isLegInSwing(leg_idx))
                    {
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "          leg_idx: " << leg_idx );

                        bot_left_pt_ = currentTransition->getNodeTo()->modeFamily->getMinPositionBaseFrame(leg_idx, foot_offset, inflated_region);
                        bot_right_pt_ = currentTransition->getNodeTo()->modeFamily->getRegionCoordinatesWorldFrame(leg_idx, oneZeroVector, foot_offset, inflated_region);
                        top_right_pt_ = currentTransition->getNodeTo()->modeFamily->getMaxPositionBaseFrame(leg_idx, foot_offset, inflated_region);

                        top_left_pt_ = bot_left_pt_ + (top_right_pt_ - bot_right_pt_);

                        mid_pt_ = (bot_left_pt_ + top_right_pt_) / 2.0;

                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_left_pt: " << bot_left_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         bot_right_pt_: " << bot_right_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         top_left_pt_: " << top_left_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         top_right_pt_: " << top_right_pt_.transpose() );
                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         mid_pt_: " << mid_pt_.transpose() );   

                        vector3_t initial_foothold = data.oMf[model.getBodyId(FootLinks.at(leg_idx))].translation();

                        // RCLCPP_INFO_STREAM(node_->get_logger(), "         initial_foothold: " << initial_foothold.transpose() );

                        // set slack_factor

                        int iter = 0;
                        int maxIters = 1000;

                        bool constraintsNotMet = true;
                        while (constraintsNotMet && iter < maxIters)
                        {
                            constraintsNotMet = false;

                            quadruped::EndEffectorLinearConstraint::Config config;
                            if (leg_idx == 0)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        FL_slack_factor);
                            else if (leg_idx == 1)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        FR_slack_factor);
                            else if (leg_idx == 2)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        BL_slack_factor);
                            else if (leg_idx == 3)
                                config = populateConfig(currTime,
                                                        bot_left_pt_,
                                                        bot_right_pt_,
                                                        top_left_pt_,
                                                        top_right_pt_,
                                                        mid_pt_,
                                                        BR_slack_factor);
                            
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "     config.Ax: " << config.Ax(0, 0) << ", " << config.Ax(0, 1) << ", " << config.Ax(0, 2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(1, 0) << ", " << config.Ax(1, 1) << ", " << config.Ax(1, 2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(2, 0) << ", " << config.Ax(2, 1) << ", " << config.Ax(2, 2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.Ax(3, 0) << ", " << config.Ax(3, 1) << ", " << config.Ax(3, 2) << "\n";

                            // RCLCPP_INFO_STREAM(node_->get_logger(), "     config.b : " << config.b(0) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.b(1) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.b(2) << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "                " << config.b(3) << "\n";

                            vector_t initialValue = config.Ax * initial_foothold + config.b;

                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "    initialValue: " << initialValue[0] << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "                  " << initialValue[1] << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "                  " << initialValue[2] << "\n";
                            // RCLCPP_INFO_STREAM(node_->get_logger(),  "                  " << initialValue[3] << "\n";

                            for (int i = 0; i < 4; i++)
                            {
                                if (initialValue[i] < 0.0)
                                {
                                    constraintsNotMet = true;
                                    
                                    if (leg_idx == 0)
                                        FL_slack_factor += 0.1;
                                    else if (leg_idx == 1)
                                        FR_slack_factor += 0.1;
                                    else if (leg_idx == 2)
                                        BL_slack_factor += 0.1;
                                    else if (leg_idx == 3)
                                        BR_slack_factor += 0.1;                                                                                                                        

                                    break;
                                }
                            }
                            iter++;
                        } 

                        if (leg_idx == 0)
                        {
                            FL_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial FL_slack_factor: " << FL_slack_factor );
                        } else if (leg_idx == 1)
                        {
                            FR_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial FR_slack_factor: " << FR_slack_factor );
                        } else if (leg_idx == 2)
                        {
                            BL_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial BL_slack_factor: " << BL_slack_factor );
                        } else if (leg_idx == 3)
                        {
                            BR_slack_factor *= 1.25; // once we find the value that starts us off within polygon, inflate it a bit more to give some wiggle room.
                            // RCLCPP_INFO_STREAM(node_->get_logger(), "      initial BR_slack_factor: " << BR_slack_factor );
                        }
                    }
                }       
            } else if (currTime >= 0.05 && currTime < 0.45) // swing time
            {
                // build lines
                visualizeShrinkingPolygons(lead, step_counter, currTime);
            }

            const auto timeStamp = ros::Time::now();

            go2Visualizer_->publishObservation(timeStamp, sol);
            ros::Rate(real_time_factor * 1.0 / (nextTime - currTime)).sleep();      
            
            // if (indx == 0)
            //     ros::Duration(5.0).sleep();
        }
    }
}
*/

void MultiModalVisualizer::visualizePathNTimes(const vector_array_t & state_trajectory, 
                                                    const vector_array_t & input_trajectory,
                                                    const scalar_array_t & time_trajectory,                                                      
                                                    double real_time_factor)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[visualizePathNTimes]" );

    if (state_trajectory.size() == 0)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "  path is size 0! returning..." );
        return;   
    }
    std::string input;

    RCLCPP_INFO_STREAM(node_->get_logger(), "Press G to start visualizing..." );
    std::cin >> input;

    while (input == "g")
    {
        for (size_t indx = 0; indx < (state_trajectory.size()-1); indx++) 
        {
            ocs2::SystemObservation sol;
            sol.state = state_trajectory[indx];
            sol.input = input_trajectory[indx];
            double currTime = time_trajectory[indx];
            double nextTime = time_trajectory[indx+1];         

            const auto timeStamp = node_->get_clock()->now();

            go2Visualizer_->publishObservation(timeStamp, sol);
            rclcpp::Rate(real_time_factor * 1.0 / (nextTime - currTime)).sleep();      
            
            // if (indx == 0)
            //     ros::Duration(5.0).sleep();
        }

        RCLCPP_INFO_STREAM(node_->get_logger(), "Press G to continue visualizing, press another key to stop." );
        std::cin >> input;
    }
}

void MultiModalVisualizer::visualizePath(const vector_array_t & state_trajectory, 
                                         const vector_array_t & input_trajectory,
                                         const scalar_array_t & time_trajectory,                                                      
                                         double real_time_factor,
                                         double num_path_visualizations)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "[visualizePath]" );

    if (state_trajectory.size() == 0)
        return;

    for (size_t path_vis = 0; path_vis < num_path_visualizations; path_vis++) 
    {
        for (size_t indx = 0; indx < (state_trajectory.size()-1); indx++) 
        {
            ocs2::SystemObservation sol;
            sol.state = state_trajectory[indx];
            sol.input = input_trajectory[indx];
            double currTime = time_trajectory[indx];
            double nextTime = time_trajectory[indx+1];         

            const auto timeStamp = node_->get_clock()->now();

            go2Visualizer_->publishObservation(timeStamp, sol);
            rclcpp::Rate(real_time_factor * 1.0 / (nextTime - currTime)).sleep();      
            
            // if (indx == 0)
            //     ros::Duration(5.0).sleep();
        }
    }
}

}  // namespace mmp


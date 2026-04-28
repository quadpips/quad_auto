#include <mmp_quadruped/TerrainPublisher.h>

namespace mmp {

TerrainPublisher::TerrainPublisher(const rclcpp::Node::SharedPtr& node, 
                                    const std::string & envFileString)
{
  // std::cout << "[TerrainPublisher()]" << std::endl;

  // nodeHandle_ =  nodeHandle;
  node_ = node;
  envFileString_ = envFileString;

  // parse the environment file
  parseEnvJson();

  // set up terrain publisher
  terrainPub_ = node_->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
                                        "/convex_plane_decomposition_ros/planar_terrain", 1);

  // observation subscriber
  std::string mpc_topic_prefix = "go2";
  auto observation_callback = 
      [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) 
      {
        // std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
        // Eigen::VectorXd latest_state = latestObservation_.state;
        // latest_q = latest_state.tail<18>();

        RCLCPP_INFO_STREAM(node_->get_logger(), "Received new observation message.");

        latest_x = latestObservation_.state;

        readyToPublish_ = true;
      };
  observationSub_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(mpc_topic_prefix 
      + "_mpc_observation", 1, observation_callback);

  localRegionPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/ground_truth/planar_regions", 1);
  localRegionNormalPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/ground_truth/planar_region_normals", 1);
  localRegionIDPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/ground_truth/ids", 1);

}

void TerrainPublisher::parseEnvJson()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Parsing environment file: " << envFileString_);
  // std::cout << "[parseEnvJson()]" << std::endl;

  // parse the environment file and store all PlanarRegions
  std::ifstream env_fstream(envFileString_);
  env_fstream >> envJson;

  auto env_regions = envJson["stone_setup"];

  int counter = 0;

  // ROS_INFO_STREAM("   publishing regions... ");

  // Here, we are exploiting the assumption that these regions are all quadrilaterals
  for (auto s : env_regions["poses"]) 
  {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "     region: " << counter);

    auto pose = env_regions["poses"][counter];
    auto scale = env_regions["scales"][counter];
    auto color = env_regions["colors"][counter];

    vector3_t stone_center_pt(pose[0].asDouble(), pose[1].asDouble(), pose[2].asDouble());
    vector3_t stone_scale(scale[0].asDouble(), scale[1].asDouble(), scale[2].asDouble());
    vector3_t stone_rpy(pose[3].asDouble(), pose[4].asDouble(), pose[5].asDouble()); // roll, pitch, yaw

    double width = stone_scale[0];
    double length = stone_scale[1];
    double height = stone_scale[2];

    // compute plane normal and basis vectors
    tf2::Quaternion q_stone;
    q_stone.setRPY(stone_rpy[0], stone_rpy[1], stone_rpy[2]); // roll, pitch, yaw
    tf2::Matrix3x3 m_stone(q_stone);

    // create a PlanarRegion
    convex_plane_decomposition::PlanarRegion region;
    
    // transform
    region.transformPlaneToWorld.translation() = stone_center_pt;
    region.transformPlaneToWorld.linear().col(0) = vector3_t(m_stone.getColumn(0).x(), 
                                                              m_stone.getColumn(0).y(), 
                                                              m_stone.getColumn(0).z());
    region.transformPlaneToWorld.linear().col(1) = vector3_t(m_stone.getColumn(1).x(), 
                                                              m_stone.getColumn(1).y(), 
                                                              m_stone.getColumn(1).z());
    region.transformPlaneToWorld.linear().col(2) = vector3_t(m_stone.getColumn(2).x(), 
                                                              m_stone.getColumn(2).y(), 
                                                              m_stone.getColumn(2).z());

    // raise by half height to account for center being in middle of stone
    region.transformPlaneToWorld.translation() += (height / 2.0) * region.transformPlaneToWorld.linear().col(2);

    // ROS_INFO_STREAM("     region.transformPlaneToWorld.translation(): " << region.transformPlaneToWorld.translation().transpose());
    // ROS_INFO_STREAM("     region.transformPlaneToWorld.linear(): " << region.transformPlaneToWorld.linear().row(0));
    // ROS_INFO_STREAM("                                            " << region.transformPlaneToWorld.linear().row(1));
    // ROS_INFO_STREAM("                                            " << region.transformPlaneToWorld.linear().row(2));

    // boundaryWithInset (in terrain frame)
    convex_plane_decomposition::BoundaryWithInset boundaryWithInset;

    convex_plane_decomposition::CgalPolygonWithHoles2d polygonWithHoles;
    convex_plane_decomposition::CgalPolygon2d polygon;
    polygon.container().emplace_back(-width / 2.0, -length / 2.0); // bottom left
    polygon.container().emplace_back(width / 2.0, -length / 2.0); // bottom right
    polygon.container().emplace_back(width / 2.0, length / 2.0); // top right
    polygon.container().emplace_back(-width / 2.0, length / 2.0); // top left

    polygonWithHoles.outer_boundary() = polygon;

    boundaryWithInset.boundary = polygonWithHoles;

    // try to inflate
    std::vector<convex_plane_decomposition::CgalPolygonWithHoles2d> insets;

    if (std::min(width, length) >= FOOT_RADIUS)
    {
      // inflate
      convex_plane_decomposition::CgalPolygon2d inflated_polygon;
      inflated_polygon.container().emplace_back(-width / 2.0 + FOOT_RADIUS, -length / 2.0 + FOOT_RADIUS); // bottom left
      inflated_polygon.container().emplace_back(width / 2.0 - FOOT_RADIUS, -length / 2.0 + FOOT_RADIUS); // bottom right
      inflated_polygon.container().emplace_back(width / 2.0 - FOOT_RADIUS, length / 2.0 - FOOT_RADIUS); // top right
      inflated_polygon.container().emplace_back(-width / 2.0 + FOOT_RADIUS, length / 2.0 - FOOT_RADIUS); // top left

      convex_plane_decomposition::CgalPolygonWithHoles2d inflated_polygon_with_holes;
      inflated_polygon_with_holes.outer_boundary() = inflated_polygon;

      insets.push_back(inflated_polygon_with_holes);

    } else
    {
      // do not inflate
    }
    boundaryWithInset.insets = insets;

    if (!boundaryWithInset.insets.empty())
    {
      region.boundaryWithInset = boundaryWithInset;
      region.bbox2d = boundaryWithInset.boundary.outer_boundary().bbox();
      allPlanarRegions_.push_back(region);
    }
  
    counter++;
  }
}

void TerrainPublisher::publishTerrain()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "[TerrainPublisher::publishTerrain()]");

  if (!readyToPublish_)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "TerrainPublisher not ready to publish yet.");
    return;
  }

  std::vector<convex_plane_decomposition::PlanarRegion> regionsToPublish;
  convex_plane_decomposition_msgs::msg::PlanarTerrain terrain_msg;

  for (int regionID = 0; regionID < allPlanarRegions_.size(); regionID++)
  {
    // std::cout << "regionID: " << regionID << std::endl;

    convex_plane_decomposition::PlanarRegion region = allPlanarRegions_[regionID];

    // check if region is within local radius
    vector3_t region_center = region.transformPlaneToWorld.translation();

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    vector3_t torso_position = extractTorsoPosition(torso_pose);
    double region_dist = (torso_position - region_center).norm();
    double local_env_radius = 1.25; // meters

    if (region_dist > local_env_radius)
    {
      // std::cout << "  skipping region, too far away: " << region_dist << std::endl;
      continue;
    }

    // std::cout << "adding region: " << region_center.transpose() << std::endl;

    regionsToPublish.push_back(region);

    convex_plane_decomposition_msgs::msg::PlanarRegion region_msg = convex_plane_decomposition::toMessage(region);

    terrain_msg.planar_regions.push_back(region_msg);
  }

  // placeholder gridMap
  grid_map::GridMap grid_map;
  grid_map::Length grid_map_dimensions(1.0, 1.0); // lengths in x,y directions [m]
  double grid_map_resolution = 0.1; // resolution [m]
  grid_map::Position grid_map_origin(0.0, 0.0); // origin [m]
  grid_map.setGeometry(grid_map_dimensions, 
                        grid_map_resolution, 
                        grid_map_origin);
  grid_map.add("elevation", 0.0); // add layer with value to initialize to everywhere'
  grid_map.setFrameId("odom");

  grid_map_msgs::msg::GridMap grid_map_msg = *(grid_map::GridMapRosConverter::toMessage(grid_map));
  terrain_msg.gridmap = grid_map_msg;

  terrainPub_->publish(terrain_msg);

  // visualize regions
  std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> terrainPtr = std::make_unique<switched_model::SegmentedPlanesTerrainModel>(convex_plane_decomposition::fromMessage(terrain_msg));

    visualizePlanarRegionBoundaries(terrainPtr, regionsToPublish);
    visualizePlanarRegionNormals(terrainPtr, regionsToPublish);
    visualizePlanarRegionIDs(terrainPtr, regionsToPublish);  
}

void TerrainPublisher::visualizePlanarRegionBoundaries(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                                        const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions)
{
    int counter = 0;
    visualization_msgs::msg::MarkerArray planarRegionMarkerArray;

    for (int i = 0; i < (int) planarRegions.size(); i++)
    {
        convex_plane_decomposition::PlanarRegion region = planarRegions[i];

        rclcpp::Time timeStamp = node_->get_clock()->now();

        switched_model::ConvexTerrain convexTerrain = terrainPtr->getConvexTerrainAtPositionInWorld(region.transformPlaneToWorld.translation(),
            [](const Eigen::Vector3d&) { return 0.0; });

        // Add region marker
        visualization_msgs::msg::Marker regionMarker;
        regionMarker.header.frame_id = "odom";
        regionMarker.header.stamp = timeStamp;
        regionMarker.ns = "local_regions";
        regionMarker.id = i;

        std::vector<geometry_msgs::msg::Point> boundary;
        boundary.reserve(convexTerrain.boundary.size() + 1);
    
        for (const auto& point : convexTerrain.boundary) 
        {
            const auto& pointInWorldFrame = switched_model::positionInWorldFrameFromPositionInTerrain({point.x(), point.y(), 0.0}, convexTerrain.plane);
            geometry_msgs::msg::Point pointMsg;
            pointMsg.x = pointInWorldFrame.x();
            pointMsg.y = pointInWorldFrame.y();
            pointMsg.z = pointInWorldFrame.z();
            boundary.emplace_back(pointMsg);
        }
    
        // Close the polygon
        const auto& pointInWorldFrame = switched_model::positionInWorldFrameFromPositionInTerrain(
            {convexTerrain.boundary.front().x(), convexTerrain.boundary.front().y(), 0.0}, convexTerrain.plane);
        geometry_msgs::msg::Point pointMsg;
        pointMsg.x = pointInWorldFrame.x();
        pointMsg.y = pointInWorldFrame.y();
        pointMsg.z = pointInWorldFrame.z();
        boundary.emplace_back(pointMsg);

        // Headers
        regionMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        regionMarker.action = visualization_msgs::msg::Marker::ADD;
        regionMarker.scale.x = 0.02; // Line width
        regionMarker.color.a = 1.0; // Opacity
        regionMarker.color.r = 0.0; // Red
        regionMarker.color.g = 0.4470; // Green
        regionMarker.color.b = 0.7410; // Blue
        regionMarker.points = boundary;
        regionMarker.pose.orientation.w = 1.0; // No rotation
        regionMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
        planarRegionMarkerArray.markers.emplace_back(regionMarker);
    }

    if (planarRegions.size() < priorPlanarRegionsSize)
    {
        // Clear extra markers from prior visualization
        for (size_t j = planarRegions.size(); j < priorPlanarRegionsSize; j++)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );

            rclcpp::Time timeStamp = node_->get_clock()->now();

            // Add region marker
            visualization_msgs::msg::Marker regionMarker;
            regionMarker.header.frame_id = "odom";
            regionMarker.header.stamp = timeStamp;
            regionMarker.ns = "local_regions";
            regionMarker.id = j;
            regionMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            regionMarker.action = visualization_msgs::msg::Marker::DELETE;
            regionMarker.pose.position.x = 0.0;
            regionMarker.pose.position.y = 0.0;
            regionMarker.pose.position.z = 0.0;
            regionMarker.pose.orientation.w = 1.0; // No rotation
            regionMarker.text = ""; // Region ID as text
            // regionMarker.scale.x = 1.0; // radius
            // regionMarker.scale.y = 1.0; // radius
            regionMarker.scale.z = 0.05; // radius
            regionMarker.color.a = 1.0; // transparency
            regionMarker.color.r = 0.0; // red
            regionMarker.color.g = 0.0; // green
            regionMarker.color.b = 0.0; // blue
            regionMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
            planarRegionMarkerArray.markers.push_back(regionMarker);
        }
    }

    priorPlanarRegionsSize = planarRegions.size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing planar region markers of size " << planarRegionMarkerArray.markers.size());
    localRegionPublisher_->publish(planarRegionMarkerArray);
}

void TerrainPublisher::visualizePlanarRegionNormals(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                                const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions)
{
    int counter = 0;
    visualization_msgs::msg::MarkerArray planarRegionNormalMarkerArray;

    for (int i = 0; i < (int) planarRegions.size(); i++)
    {
        convex_plane_decomposition::PlanarRegion region = planarRegions[i];

        rclcpp::Time timeStamp = node_->get_clock()->now();

        switched_model::ConvexTerrain convexTerrain = terrainPtr->getConvexTerrainAtPositionInWorld(region.transformPlaneToWorld.translation(),
            [](const Eigen::Vector3d&) { return 0.0; });

        // Add region ID marker
        visualization_msgs::msg::Marker regionNormalMarker;
        regionNormalMarker.header.frame_id = "odom";
        regionNormalMarker.header.stamp = timeStamp;
        regionNormalMarker.ns = "local_region_normals";
        regionNormalMarker.id = i;
        regionNormalMarker.type = visualization_msgs::msg::Marker::ARROW;
        regionNormalMarker.action = visualization_msgs::msg::Marker::ADD;
        regionNormalMarker.scale.x = 0.01;
        regionNormalMarker.scale.y = 0.02;
        regionNormalMarker.scale.z = 0.06;
        regionNormalMarker.points.reserve(2);
        double normalLength = 0.1;
        const Eigen::Vector3d surfaceNormal = normalLength * surfaceNormalInWorld(convexTerrain.plane);
        const Eigen::Vector3d startPointVec = convexTerrain.plane.positionInWorld;
        geometry_msgs::msg::Point startPoint;
        startPoint.x = startPointVec.x();
        startPoint.y = startPointVec.y();
        startPoint.z = startPointVec.z();
        geometry_msgs::msg::Point endPoint;
        endPoint.x = startPointVec.x() + surfaceNormal.x();
        endPoint.y = startPointVec.y() + surfaceNormal.y();
        endPoint.z = startPointVec.z() + surfaceNormal.z();
        regionNormalMarker.points.push_back(startPoint);
        regionNormalMarker.points.push_back(endPoint);
        regionNormalMarker.color.a = 1.0; // transparency
        regionNormalMarker.color.r = 0.0; // red
        regionNormalMarker.color.g = 0.4470; // green
        regionNormalMarker.color.b = 0.7410; // blue
        regionNormalMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
        regionNormalMarker.pose.orientation.w = 1.0; // No rotation
        planarRegionNormalMarkerArray.markers.push_back(regionNormalMarker);
    }

    if (planarRegions.size() < priorPlanarRegionsIDSize)
    {
        // Clear extra markers from prior visualization
        for (size_t j = planarRegions.size(); j < priorPlanarRegionsIDSize; j++)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );

            rclcpp::Time timeStamp = node_->get_clock()->now();

            // Add region ID marker
            visualization_msgs::msg::Marker regionNormalMarker;
            regionNormalMarker.header.frame_id = "odom";
            regionNormalMarker.header.stamp = timeStamp;
            regionNormalMarker.ns = "local_region_normals";
            regionNormalMarker.id = j;
            regionNormalMarker.type = visualization_msgs::msg::Marker::ARROW;
            regionNormalMarker.action = visualization_msgs::msg::Marker::DELETE;
            regionNormalMarker.pose.position.x = 0.0;
            regionNormalMarker.pose.position.y = 0.0;
            regionNormalMarker.pose.position.z = 0.0;
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );
            regionNormalMarker.pose.orientation.w = 1.0; // No rotation
            regionNormalMarker.text = ""; // Region ID as text
            // regionNormalMarker.scale.x = 1.0; // radius
            // regionNormalMarker.scale.y = 1.0; // radius
            regionNormalMarker.scale.z = 0.05; // radius
            regionNormalMarker.color.a = 1.0; // transparency
            regionNormalMarker.color.r = 0.0; // red
            regionNormalMarker.color.g = 0.0; // green
            regionNormalMarker.color.b = 0.0; // blue
            regionNormalMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
            planarRegionNormalMarkerArray.markers.push_back(regionNormalMarker);
        }
    }

    priorPlanarRegionsNormalSize = planarRegions.size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing planar region normal markers of size " << planarRegionNormalMarkerArray.markers.size());
    localRegionNormalPublisher_->publish(planarRegionNormalMarkerArray);
}

void TerrainPublisher::visualizePlanarRegionIDs(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                            const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions)
{
    int counter = 0;
    visualization_msgs::msg::MarkerArray planarRegionIDMarkerArray;

    for (int i = 0; i < (int) planarRegions.size(); i++)
    {
        convex_plane_decomposition::PlanarRegion region = planarRegions[i];

        rclcpp::Time timeStamp = node_->get_clock()->now();

        switched_model::ConvexTerrain convexTerrain = terrainPtr->getConvexTerrainAtPositionInWorld(region.transformPlaneToWorld.translation(),
            [](const Eigen::Vector3d&) { return 0.0; });

        // Add region ID marker
        visualization_msgs::msg::Marker regionIDMarker;
        regionIDMarker.header.frame_id = "odom";
        regionIDMarker.header.stamp = timeStamp;
        regionIDMarker.ns = "local_region_ids";
        regionIDMarker.id = i;
        regionIDMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        regionIDMarker.action = visualization_msgs::msg::Marker::ADD;
        regionIDMarker.pose.position.x = convexTerrain.plane.positionInWorld.x();
        regionIDMarker.pose.position.y = convexTerrain.plane.positionInWorld.y();
        regionIDMarker.pose.position.z = convexTerrain.plane.positionInWorld.z();
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Region " << i << " position: (" << regionIDMarker.pose.position.x << ", " << regionIDMarker.pose.position.y << ", " << regionIDMarker.pose.position.z << ")");
        regionIDMarker.pose.orientation.w = 1.0; // No rotation
        regionIDMarker.text = std::to_string(i); // Region ID as text
        // regionIDMarker.scale.x = 1.0; // radius
        // regionIDMarker.scale.y = 1.0; // radius
        regionIDMarker.scale.z = 0.05; // radius
        regionIDMarker.color.a = 1.0; // transparency
        regionIDMarker.color.r = 0.0; // red
        regionIDMarker.color.g = 0.0; // green
        regionIDMarker.color.b = 0.0; // blue
        regionIDMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
        planarRegionIDMarkerArray.markers.push_back(regionIDMarker);
    }

    if (planarRegions.size() < priorPlanarRegionsIDSize)
    {
        // Clear extra markers from prior visualization
        for (size_t j = planarRegions.size(); j < priorPlanarRegionsIDSize; j++)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );

            rclcpp::Time timeStamp = node_->get_clock()->now();

            // Add region ID marker
            visualization_msgs::msg::Marker regionIDMarker;
            regionIDMarker.header.frame_id = "odom";
            regionIDMarker.header.stamp = timeStamp;
            regionIDMarker.ns = "local_region_ids";
            regionIDMarker.id = j;
            regionIDMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            regionIDMarker.action = visualization_msgs::msg::Marker::DELETE;
            regionIDMarker.pose.position.x = 0.0;
            regionIDMarker.pose.position.y = 0.0;
            regionIDMarker.pose.position.z = 0.0;
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );
            regionIDMarker.pose.orientation.w = 1.0; // No rotation
            regionIDMarker.text = ""; // Region ID as text
            // regionIDMarker.scale.x = 1.0; // radius
            // regionIDMarker.scale.y = 1.0; // radius
            regionIDMarker.scale.z = 0.05; // radius
            regionIDMarker.color.a = 1.0; // transparency
            regionIDMarker.color.r = 0.0; // red
            regionIDMarker.color.g = 0.0; // green
            regionIDMarker.color.b = 0.0; // blue
            regionIDMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
            planarRegionIDMarkerArray.markers.push_back(regionIDMarker);
        }
    }

    priorPlanarRegionsIDSize = planarRegions.size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing planar region ID markers of size " << planarRegionIDMarkerArray.markers.size());
    localRegionIDPublisher_->publish(planarRegionIDMarkerArray);    
}


}  // namespace mmp

//
// Created by rgrandia on 24.06.20.
//

#include "segmented_planes_terrain_model/SegmentedPlanesTerrainVisualization.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <random>

namespace switched_model {

// visualization_msgs::msg::MarkerArray getConvexTerrainMarkers(const ConvexTerrain& convexTerrain, 
//                                                               const std_msgs::msg::ColorRGBA & color, 
//                                                               const rclcpp::Time & timeStamp,                                                              
//                                                               double linewidth,
//                                                               double normalLength) {
//   visualization_msgs::msg::MarkerArray markerArray;
//   markerArray.markers.reserve(2);

//   addConvexTerrainMarkerToMarkerArray(markerArray, convexTerrain, color, timeStamp, linewidth, normalLength);

//   return markerArray;
// }

void addConvexTerrainMarkerToMarkerArray(visualization_msgs::msg::MarkerArray & markerArray, 
                                          const ConvexTerrain& convexTerrain, 
                                          // const std_msgs::msg::ColorRGBA & color, 
                                          const rclcpp::Time & timeStamp,
                                          double linewidth,
                                          double normalLength)
{
  ocs2::Color ocs2Color = ocs2::Color::blue;

  std::string frameId = "odom";

  // needlessly complicated, I just want the same regions to keep the same color across time
  // std::string regionID = std::to_string(convexTerrain.plane.positionInWorld.x()) + "_" +
  //                         std::to_string(convexTerrain.plane.positionInWorld.y()) +  "_" +
  //                         std::to_string(convexTerrain.plane.positionInWorld.z());

  // std::default_random_engine generator;
  // generator.seed(std::hash<std::string>{}(regionID));
  // std::uniform_real_distribution<double> distribution(0.0,1.0);

  // Define new color for region to keep constant color for each region
  // std_msgs::msg::ColorRGBA regionColor;
  // regionColor.r = color.r;
  // regionColor.g = color.g; // distribution(generator);
  // regionColor.b = color.b; // distribution(generator);
  // regionColor.a = 1.0;

  // Mark the surface normal
  const vector3_t surfaceNormal = normalLength * surfaceNormalInWorld(convexTerrain.plane);

  // std::cout << "    surfaceNormal = " << surfaceNormal.transpose() << std::endl;

  auto normalMsg = ocs2::getArrowAtPointMsg(surfaceNormal, convexTerrain.plane.positionInWorld, ocs2Color);
  // normalMsg.lifetime = rclcpp::Duration::from_seconds(0.0);
  normalMsg.color = getColor(ocs2Color);  // regionColor;
  normalMsg.header = ocs2::getHeaderMsg(frameId, timeStamp);
  normalMsg.id = markerArray.markers.size();
  normalMsg.lifetime = rclcpp::Duration::from_seconds(0.0);

  markerArray.markers.emplace_back(normalMsg);

  // Polygon message
  if (!convexTerrain.boundary.empty()) 
  {
    std::vector<geometry_msgs::msg::Point> boundary;
    boundary.reserve(convexTerrain.boundary.size() + 1);
  
    for (const auto& point : convexTerrain.boundary) 
    {
      const auto& pointInWorldFrame = positionInWorldFrameFromPositionInTerrain({point.x(), point.y(), 0.0}, convexTerrain.plane);
      auto pointMsg = ocs2::getPointMsg(pointInWorldFrame);
      boundary.emplace_back(pointMsg);
    }
  
    // Close the polygon
    const auto& pointInWorldFrame = positionInWorldFrameFromPositionInTerrain(
        {convexTerrain.boundary.front().x(), convexTerrain.boundary.front().y(), 0.0}, convexTerrain.plane);
    auto pointMsg = ocs2::getPointMsg(pointInWorldFrame);
    boundary.emplace_back(pointMsg);

    // Headers

    auto lineMsg = ocs2::getLineMsg(std::move(boundary), ocs2Color, linewidth);
    lineMsg.header = ocs2::getHeaderMsg(frameId, timeStamp);
    lineMsg.id = markerArray.markers.size();
    // lineMsg.color = regionColor;
    lineMsg.lifetime = rclcpp::Duration::from_seconds(0.0);

    markerArray.markers.emplace_back(lineMsg);
  }
}

}  // namespace switched_model

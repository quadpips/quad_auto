//
// Created by rgrandia on 23.06.20.
//

#include "segmented_planes_terrain_model/SegmentedPlanesTerrainModel.h"

#include <algorithm>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

#include <grid_map_filters_rsl/lookup.hpp>

namespace switched_model {

namespace {
const std::string elevationLayerName = "elevation";
}  // namespace

SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(convex_plane_decomposition::PlanarTerrain planarTerrain)
    : planarTerrain_(std::move(planarTerrain)),
      signedDistanceField_(nullptr),
      elevationData_(&planarTerrain_.gridMap.get(elevationLayerName)) 
{
  // std::cout << "          [SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel()]" << std::endl;
  // std::cout << "            planarTerrain_.planarRegions.size(): " << planarTerrain_.planarRegions.size() << std::endl;
}

TerrainPlane SegmentedPlanesTerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t& positionInWorld, std::function<scalar_t(const vector3_t&)> penaltyFunction) const 
{
  // std::cout << "          [SegmentedPlanesTerrainModel::getLocalTerrainAtPositionInWorldAlongGravity]" << std::endl;

  // std::cout << "            planarTerrain_.planarRegions.size(): " << planarTerrain_.planarRegions.size() << std::endl;


  const auto projection = getBestPlanarRegionAtPositionInWorld(positionInWorld, planarTerrain_.planarRegions, std::move(penaltyFunction));
  if (projection.regionPtr == nullptr) {
    throw std::runtime_error("[SegmentedPlanesTerrainModel] no region found");
  }

  // std::cout << "            projection.positionInWorld: " << projection.positionInWorld.transpose() << std::endl;
  // std::cout << "            projection.positionInTerrainFrame: (" << projection.positionInTerrainFrame.x() << ", " << projection.positionInTerrainFrame.y() << ")" << std::endl;

  return TerrainPlane{projection.positionInWorld, projection.regionPtr->transformPlaneToWorld.linear().transpose()};
}

ConvexTerrain SegmentedPlanesTerrainModel::getConvexTerrainAtPositionInWorld(
    const vector3_t& positionInWorld, std::function<scalar_t(const vector3_t&)> penaltyFunction) const 
{
  // std::cout << "          [SegmentedPlanesTerrainModel::getConvexTerrainAtPositionInWorld]" << std::endl;
  // std::cout << "            positionInWorld: " << positionInWorld.transpose() << std::endl;

  // std::cout << "            planarTerrain_.planarRegions.size(): " << planarTerrain_.planarRegions.size() << std::endl;

  const auto projection = getBestPlanarRegionAtPositionInWorld(positionInWorld, planarTerrain_.planarRegions, std::move(penaltyFunction));
  if (projection.regionPtr == nullptr) {
    throw std::runtime_error("[SegmentedPlanesTerrainModel] no region found");
  }

  // std::cout << "            projection.positionInWorld: " << projection.positionInWorld.transpose() << std::endl;
  // std::cout << "            projection.positionInTerrainFrame: (" << projection.positionInTerrainFrame.x() << ", " << projection.positionInTerrainFrame.y() << ")" << std::endl;


  // Convert boundary and projection to terrain frame
  const int numberOfVertices = 16;  // Multiple of 4 is nice for symmetry.
  const double growthFactor = 1.05;
  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
      projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numberOfVertices, growthFactor);

  // Return convex region with origin at the projection
  ConvexTerrain convexTerrain;
  convexTerrain.plane = {projection.positionInWorld,
                         projection.regionPtr->transformPlaneToWorld.linear().transpose()};  // Origin is at the projection
  convexTerrain.boundary.reserve(convexRegion.size());
  for (const auto& point : convexRegion) {
    convexTerrain.boundary.emplace_back(point.x() - projection.positionInTerrainFrame.x(),
                                        point.y() - projection.positionInTerrainFrame.y());  // Shift points to new origin
  }
  return convexTerrain;
}

void SegmentedPlanesTerrainModel::createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates,
                                                              const Eigen::Vector3d& maxCoordinates) {
  // Compute coordinates of submap
  const auto minXY =
      grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap, grid_map::Position(minCoordinates.x(), minCoordinates.y()));
  const auto maxXY =
      grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap, grid_map::Position(maxCoordinates.x(), maxCoordinates.y()));
  const auto centerXY = 0.5 * (minXY + maxXY);
  const auto lengths = maxXY - minXY;

  bool success = true;
  grid_map::GridMap subMap = planarTerrain_.gridMap.getSubmap(centerXY, lengths, success);
  if (success) {
    signedDistanceField_ =
        std::make_unique<SegmentedPlanesSignedDistanceField>(subMap, elevationLayerName, minCoordinates.z(), maxCoordinates.z());
  } else {
    std::cerr << "[SegmentedPlanesTerrainModel] Failed to get subMap" << std::endl;
  }
}

// vector3_t SegmentedPlanesTerrainModel::getHighestObstacleAlongLine(const vector3_t& position1InWorld,
//                                                                    const vector3_t& position2InWorld) const {
//   const auto result = grid_map::lookup::maxValueBetweenLocations(
//       {position1InWorld.x(), position1InWorld.y()}, {position2InWorld.x(), position2InWorld.y()}, planarTerrain_.gridMap, *elevationData_);
//   if (result.isValid) {
//     return {result.position.x(), result.position.y(), result.value};
//   } else {
//     // return highest query point if the map didn't work.
//     if (position1InWorld.z() > position2InWorld.z()) {
//       return position1InWorld;
//     } else {
//       return position2InWorld;
//     }
//   }
// }

std::vector<vector2_t> SegmentedPlanesTerrainModel::getHeightProfileAlongLine(const vector3_t& position1InWorld,
                                                                              const vector3_t& position2InWorld) const 
{
  // std::cout << "[SegmentedPlanesTerrainModel::getHeightProfileAlongLine]" << std::endl;

  const vector2_t diff2d = position2InWorld.head<2>() - position1InWorld.head<2>();
  const scalar_t diffSquareNorm = diff2d.squaredNorm();
  const auto resolution = planarTerrain_.gridMap.getResolution();

  if (diffSquareNorm > (resolution * resolution))  // norm(p2-p1)_XY > resolution
  {
    // std::cout << "    case 1 " << std::endl;

    const auto pointsOnLine =
        grid_map::lookup::valuesBetweenLocations({position1InWorld.x(), position1InWorld.y()}, {position2InWorld.x(), position2InWorld.y()},
                                                 planarTerrain_.gridMap, *elevationData_);

    std::vector<vector2_t> heightProfile;
    heightProfile.reserve(pointsOnLine.size());

    for (const auto& point : pointsOnLine) {
      const vector2_t pointDiff = point.head<2>() - position1InWorld.head<2>();
      const scalar_t lineProgress = pointDiff.dot(diff2d) / diffSquareNorm;
      if (lineProgress >= 0.0 && lineProgress <= 1.0) {
        heightProfile.push_back({lineProgress, point.z()});
      }
    }

    return heightProfile;
  } else
  {
    // std::cout << "    case 2 " << std::endl;

    grid_map::Index index;
    planarTerrain_.gridMap.getIndex({position1InWorld.x(), position1InWorld.y()}, index);
    scalar_t heightData = (*elevationData_)(index(0), index(1));
    return {{0.0, heightData}, {1.0, heightData}};
  }
}

scalar_t SegmentedPlanesTerrainModel::getHeightAtPosition(const vector3_t& positionInWorld) const
{
  grid_map::Index index;
  planarTerrain_.gridMap.getIndex({positionInWorld.x(), positionInWorld.y()}, index);
  scalar_t heightData = (*elevationData_)(index(0), index(1));
  return heightData;
}

std::vector<ConvexTerrain> SegmentedPlanesTerrainModel::getAllConvexTerrains() const
{
  // std::cout << "[SegmentedPlanesTerrainModel::getAllConvexTerrains]" << std::endl;

  std::vector<ConvexTerrain> convexTerrains;
  convexTerrains.reserve(planarTerrain_.planarRegions.size());

  for (const auto& planarRegion : planarTerrain_.planarRegions) 
  {
    const auto& convexTerrain = getConvexTerrainAtPositionInWorld(planarRegion.transformPlaneToWorld.translation(), 
                                                                  [](const vector3_t&){return 0.0;});

    convexTerrains.push_back(convexTerrain);
  }

  return convexTerrains;
}

}  // namespace switched_model

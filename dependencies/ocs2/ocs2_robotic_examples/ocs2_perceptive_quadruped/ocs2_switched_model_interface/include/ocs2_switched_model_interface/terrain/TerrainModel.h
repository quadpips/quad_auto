//
// Created by rgrandia on 21.04.20.
//

#pragma once

#include <grid_map_core/GridMap.hpp>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"
#include "ocs2_switched_model_interface/terrain/SignedDistanceField.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

/**
 * This abstract class defines the interface for terrain models.
 */
class TerrainModel {
 public:
  TerrainModel() = default;
  virtual ~TerrainModel() = default;
  TerrainModel(const TerrainModel&) = delete;
  TerrainModel& operator=(const TerrainModel&) = delete;

  /** Returns a linear approximation of the terrain at the query point projected along gravity onto the terrain  */
  // TODO: change name and remove "alongGravity"
  TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld) const 
  {
    // std::cout << "          [TerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(1)]" << std::endl;  
    return getLocalTerrainAtPositionInWorldAlongGravity(positionInWorld, [](const vector3_t&) { return 0.0; });
  }

  /// Penalty function needs to return values >= 0
  virtual TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld,
                                                                    std::function<scalar_t(const vector3_t&)> penaltyFunction) const = 0;

  ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const 
  {
    // std::cout << "          [TerrainModel::getConvexTerrainAtPositionInWorld(1)]" << std::endl;    
    return getConvexTerrainAtPositionInWorld(positionInWorld, [](const vector3_t&) { return 0.0; });
  }

  /// Penalty function needs to return values >= 0
  virtual ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld,
                                                          std::function<scalar_t(const vector3_t&)> penaltyFunction) const 
  {
    // std::cout << "          [TerrainModel::getConvexTerrainAtPositionInWorld(2)]" << std::endl;
    return {getLocalTerrainAtPositionInWorldAlongGravity(positionInWorld, std::move(penaltyFunction)), {}};
  }

  /** Returns the signed distance field for this terrain if one is available */
  virtual const SignedDistanceField* getSignedDistanceField() const { return nullptr; }

  // virtual vector3_t getHighestObstacleAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const = 0;

  /**
   * Returns the height profiles between two points in world frame. Provided as a set of points {alpha, height}, where alpha in [0, 1] is
   * the progress along the line. position1InWorld -> alpha = 0, position2InWorld -> alpha = 1.
   * Height is the absolute height in world frame.
   */
  virtual std::vector<vector2_t> getHeightProfileAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const = 0;

  virtual scalar_t getHeightAtPosition(const vector3_t& positionInWorld) const = 0;

  virtual std::vector<ConvexTerrain> getAllConvexTerrains() const = 0;

  virtual const grid_map::GridMap * getGridMap() const = 0;
  
};

}  // namespace switched_model

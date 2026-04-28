//
// Created by rgrandia on 24.06.20.
//

#pragma once

#include <ocs2_core/misc/Benchmark.h>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <mutex>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "SegmentedPlanesTerrainModel.h"
#include "SegmentedPlanesTerrainVisualization.h"

namespace switched_model {

class SegmentedPlanesTerrainModelRos {
 public:
  SegmentedPlanesTerrainModelRos(const rclcpp::Node::SharedPtr& node,
                                 const bool & vis);

  ~SegmentedPlanesTerrainModelRos();

  /// Extract the latest terrain model. Resets internal model to a nullptr
  std::unique_ptr<SegmentedPlanesTerrainModel> getTerrainModel();

  void createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates,
                                   const Eigen::Vector3d& maxCoordinates);

  void publish();
  static void toPointCloud(
      const SegmentedPlanesSignedDistanceField& segmentedPlanesSignedDistanceField,
      sensor_msgs::msg::PointCloud2& pointCloud, size_t decimation,
      const std::function<bool(float)>& condition);

 private:
    void callback(
        const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr&
            msg);

    void publishRegionIDs();

    std::pair<Eigen::Vector3d, Eigen::Vector3d> getSignedDistanceRange(
        const grid_map::GridMap& gridMap, const std::string& elevationLayer);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr 
        terrainSubscriber_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        distanceFieldPublisher_;

    // rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr
    //     mmpTerrainUpdateSubscriber_;

    std::mutex updateMutex_;
    std::atomic_bool terrainUpdated_;
    std::unique_ptr<SegmentedPlanesTerrainModel> terrainPtr_;

    std::mutex updateCoordinatesMutex_;
    Eigen::Vector3d minCoordinates_;
    Eigen::Vector3d maxCoordinates_;
    bool externalCoordinatesGiven_;

    int priorPlanarRegionsSize = 0;
    int priorPlanarRegionIDsSize = 0;

    bool vis_ = true;  // Flag to control visualization, can be set externally

    // bool mmpUpdateTerrain_ = true;

    std::mutex pointCloudMutex_;
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointCloud2MsgPtr_;

    ocs2::benchmark::RepeatedTimer callbackTimer_;
};

}  // namespace switched_model

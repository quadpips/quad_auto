#pragma once

#include <rclcpp/rclcpp.hpp>

// #include <jsoncpp/json/json.h>
#include <json/json.h>

#include <fstream>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <mmp_quadruped/planning/utils.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include <convex_plane_decomposition_ros/MessageConversion.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include "ocs2_mpc/SystemObservation.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include "segmented_planes_terrain_model/SegmentedPlanesTerrainModel.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace mmp {

class TerrainPublisher
{
    public:
        TerrainPublisher(const rclcpp::Node::SharedPtr& node, 
                            const std::string & envFileString);

        void publishTerrain();

    private:
        void parseEnvJson();

        void visualizePlanarRegionBoundaries(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                                const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions);
        void visualizePlanarRegionNormals(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                        const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions);
        void visualizePlanarRegionIDs(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                        const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions);

        std::string envFileString_;
        Json::Value envJson;

        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;

        ocs2::SystemObservation latestObservation_;
        comkino_state_t latest_x;   
        bool readyToPublish_ = false;

        int priorPlanarRegionsSize = 0;
        int priorPlanarRegionsNormalSize = 0;
        int priorPlanarRegionsIDSize = 0;

        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr terrainPub_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr localRegionPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr localRegionIDPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr localRegionNormalPublisher_;

        std::vector<convex_plane_decomposition::PlanarRegion> allPlanarRegions_;

};

}  // namespace mmp

#pragma once

#include <mmp_quadruped/planning/utils.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

// #include <convex_plane_decomposition/PlanarRegion.h>
// #include "segmented_planes_terrain_model/SegmentedPlanesTerrainModel.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

namespace mmp {

/****************************Superquadric****************************/
class Superquadric
{
    public:

        /**
        * @brief Construct a new Superquadric object
        *
        * @param leg_idx Leg index
        * @param hyperparametersFile File containing the superquadric hyperparameters
        */
        Superquadric(const rclcpp::Node::SharedPtr& node,
                        const int & leg_idx, 
                        const std::string & hyperparametersFile);

        /**
        * @brief Check if a steppable region is within the superquadric
        * 
        * @param torso_pose Torso pose to evaluate at (in base frame)
        * @param region Region (in base frame)
        * @return true if the region is within the superquadric, false if not
        */
        bool isRegionWithinSuperquadricWorldFrame(const base_coordinate_t & torso_pose, 
                                                    const switched_model::ConvexTerrain & region,
                                                    const bool & print);

        vector3_t getCenter() { return sqCenter; }
        vector3_t getOrientation() { return sqOrientation; }
        vector3_t getCurvature() { return sqCurvature; }
        vector3_t getDimension() { return sqDimension; }

    private:
        /**
        * @brief Check if a single point is within the superquadric
        *
        * @param torso_pose Torso pose to evaluate at (in base frame)
        * @param p_query Query point (in base frame)
        * @return true if the point is within the superquadric, false if not
        */
        bool isPointWithinSuperquadricWorldFrame(const base_coordinate_t & torso_pose, 
                                                const vector3_t & p_query,
                                                const bool & print);
        
        rclcpp::Node::SharedPtr node_;
                                                
        vector3_t sqCenter; /** Superquadric center point */
        vector3_t sqOrientation; /** Superquadric orientation */
        vector3_t sqCurvature; /** Superquadric curvature in three axes */
        vector3_t sqDimension; /** Superquadric dimensions in three axes */

        Eigen::Matrix4d superquadricToRbtFrame; /** Transformation matrix from superquadric frame to robot frame */
        Eigen::Matrix4d rbtToSuperquadricFrame; /** Transformation matrix from robot frame to superquadric frame */
};

}  // namespace mmp

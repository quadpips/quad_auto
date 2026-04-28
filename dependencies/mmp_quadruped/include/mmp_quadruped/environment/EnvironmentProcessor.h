#pragma once

#include <mmp_quadruped/planning/utils.h>

#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

namespace mmp {

class EnvironmentProcessor
{
    public:
        /**
         * @brief Constructor for EnvironmentProcessor object
         * 
         * @param envFile path to environment file
         * @param torsoGlobalGoalPose torso global goal pose
         */
        EnvironmentProcessor(const rclcpp::Node::SharedPtr& node,
                                std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface, 
                                const std::string & envFile,
                                base_coordinate_t & torsoInitPos,
                                base_coordinate_t & torsoGlobalGoalPose);

        /**
         * @brief Destructor for EnvironmentProcessor object
         */
        ~EnvironmentProcessor();

        /**
         * @brief extract local regions from environment
         *
         */
        void extractLocalRegions(const comkino_state_t & latest_x);

        bool readyToPlan();

        vector3_t getMinEnvPt() { return minEnvPt; }
        vector3_t getMaxEnvPt() { return maxEnvPt; }
        std::vector<switched_model::ConvexTerrain> getLocalRegions() { return localRegions_; }

        void clearRegions() { localRegions_.clear(); allRegions_.clear(); }

        // void setRegions();

    private:

        void resetMinMaxEnvPts()
        {
            minEnvPt << 1e30, 1e30, 1e30;
            maxEnvPt << -1e30, -1e30, -1e30;
        }

        // void callback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstPtr& msg);

        // /**
        // * @brief Parse environment file
        // * 
        // * @param latest_q latest robot configuration
        // */
        // void parseEnvFile(const Eigen::VectorXd & latest_q);

        // /**
        //  * @brief Parse stone file
        //  *
        //  * @param latest_q latest robot configuration
        //  */
        // void parseStoneFile(const Eigen::VectorXd & latest_q);

        /**
         * @brief update environment boundin box points to determine where to run torso path planner
         *
         * @param endpt1 end point 1 of a single region
         * @param endpt2 end point 2 of a single region
         */
        void updateMinMaxEnvPts(const vector3_t & endpt1, 
                                const vector3_t & endpt2);

        void updateMinMaxEnvPts(const vector3_t & pt);

        /**
         * @brief set torso global goal position
         *
         * @param torsoGlobalGoalPose torso global goal position
         */
        void setTorsoGlobalGoalPose(base_coordinate_t & torsoInitPos, base_coordinate_t & torsoGlobalGoalPose);

        // ros::Subscriber terrainSubscriber_;
        // std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> terrainPtr_; /**< Terrain model pointer */

        rclcpp::Node::SharedPtr node_;

        std::vector<switched_model::ConvexTerrain> allRegions_; /**< All regions in environment */
        std::vector<switched_model::ConvexTerrain> localRegions_; /**< Local regions around robot */
        
        vector3_t minEnvPt; /**< Min point of environment bounding box */
        vector3_t maxEnvPt; /**< Max point of environment bounding box */

        std::shared_ptr<switched_model::CustomQuadrupedInterface> interface_; /**< Quadruped interface */

        std::string envFile_; /**< Path to environment file */
};

}  // namespace mmp

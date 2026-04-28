#pragma once

#include <vector>
#include <mmp_quadruped/planning/ContactSequence.h>
#include <mmp_quadruped/planning/ModeFamilyNodeID.h>

#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

namespace mmp {

/****************************ModeFamily****************************/
class ModeFamily
{
    public:
        /**
        * @brief Construct a new ModeFamily object
        * 
        * @param FL_region The front left region for the mode family
        * @param FR_region The front right region for the mode family
        * @param BL_region The back left region for the mode family
        * @param BR_region The back right region for the mode family
        */
        ModeFamily(const rclcpp::Node::SharedPtr& node,
                    const ModeFamilyNodeID & nodeID,
                    const switched_model::ConvexTerrain & FL_region, 
                    const switched_model::ConvexTerrain & FR_region, 
                    const switched_model::ConvexTerrain & BL_region, 
                    const switched_model::ConvexTerrain & BR_region);


        /**
        * @brief Get the number of dimensions of the mode family
        *
        * @return The number of dimensions of the mode family
        */
        short getNumDims() const { return nDims_;}

        /**
        * @brief Get the number of dimensions of the mode family for the given leg index
        *
        * @param legIdx The index of the leg
        * @return The number of dimensions of the mode family for the given leg index
        */
        short getNumDims(const int & legIdx) const;

        /**
        * @brief Get the X position of the region for the given leg index
        *
        * @param legIdx The index of the leg
        * @return The X position of the region for the given leg index
        */
        double X(const int & legIdx) const;
        
        /**
        * @brief Get the Y position of the region for the given leg index
        *
        * @param legIdx The index of the leg
        * @return The Y position of the region for the given leg index
        */
        double Y(const int & legIdx) const;
        
        /**
        * @brief Check if foothold is on the region in base frame
        *
        * @param legIdx The index of the leg
        * @param foothold The foothold to check
        * @return True if the foothold is on the region in base frame, false otherwise
        */
        bool isFootholdOnRegionBaseFrame(const short & legIdx, const vector3_t & foothold);

        /**
        * @brief Check if foothold is on the region in base frame
        *
        * @param legIdx The index of the leg
        * @param foothold The foothold to check
        * @return True if the foothold is on the region in base frame, false otherwise
        */
        bool isFootholdOnRegionWorldFrame(const short & legIdx, const vector3_t & foothold);        

        /**
        * @brief get nominal torso position of the mode family
        *
        * @return The nominal torso position of the mode family
        */
        vector3_t getNominalTorsoPosition() const;

        // /**
        // * @brief Get goal position of goal mode family
        // */
        // vector3_t getGoalPosition() const;

        /**
        * @brief Get the ID of the region for the given leg index
        *
        * @param legIdx The index of the leg
        * @return The ID of the region for the given leg index
        */
        int getRegionID(const int & legIdx) const;

        std::string getHashID(const int & legIdx) const;

        /**
        * @brief get the coordinates of the region for the given leg index
        *
        * @param legIdx The index of the leg
        * @param foot_offset Whether to include foot offset
        * @param inflate_region Whether to inflate the region
        * @return The coordinates of the region for the given leg index
        */
        vector3_t getRegionCoordinatesBaseFrame(const int & legIdx, 
                                                    const bool & foot_offset = false,
                                                    const bool & inflate_region = false) const;

        /**
        * @brief get the coordinates of the region for the given leg index
        *
        * @param legIdx The index of the leg
        * @param foot_offset Whether to include foot offset
        * @param inflate_region Whether to inflate the region
        * @return The coordinates of the region for the given leg index
        */
        vector3_t getRegionCoordinatesWorldFrame(const int & legIdx, 
                                                    const bool & foot_offset = false,
                                                    const bool & inflate_region = false) const;

        switched_model::ConvexTerrain getRegion(const int & legIdx) const;

        // /**
        // * @brief get the minimum position of the region for the given leg index
        // *
        // * @param legIdx The index of the leg
        // * @param foot_offset Whether to include foot offset
        // * @param inflated_region Whether to inflate the region
        // * @return The minimum position of the region for the given leg index
        // */
        // vector3_t getMinPositionBaseFrame(const int & legIdx,
        //                                         const bool & foot_offset = false,
        //                                         const bool & inflated_region = false) const;

        // /**
        // * @brief get the maximum position of the region for the given leg index
        // *
        // * @param legIdx The index of the leg
        // * @param foot_offset Whether to include foot offset
        // * @param inflated_region Whether to inflate the region
        // * @return The maximum position of the region for the given leg index
        // */
        // vector3_t getMaxPositionBaseFrame(const int & legIdx,
        //                                         const bool & foot_offset = false,
        //                                         const bool & inflated_region = false) const;

        /**
        * @brief get the nominal position of the region for the given leg index in world frame
        *
        * @param legIdx The index of the leg
        * @param foot_offset Whether to include foot offset
        * @param inflated_region Whether to inflate the region
        * @return The nominal position of the region for the given leg index in world frame
        */
        vector3_t getNominalPositionWorldFrame(const int & legIdx,
                                                        const bool & foot_offset = false,
                                                        const bool & inflated_region = false) const;

        // /**
        // * @brief get the nominal position of the region for the given leg index in base frame
        // *
        // * @param legIdx The index of the leg
        // * @param foot_offset Whether to include foot offset
        // * @param inflated_region Whether to inflate the region
        // * @return The nominal position of the region for the given leg index in base frame
        // */
        // vector3_t getNominalPositionBaseFrame(const int & legIdx,
        //                                             const bool & foot_offset = false,
        //                                             const bool & inflated_region = false) const;

        /**
        * @brief get the normal of the region for the given leg index in base frame
        *
        * @param legIdx The index of the leg
        * @return The normal of the region for the given leg index in base frame
        */
        vector3_t getNormalWorldFrame(const int & legIdx) const;

        /**
        * @brief get the phase string that describes this mode family
        *
        * @return The phase string that describes this mode family
        */
        std::string getPhase();

        /**
        * @brief check if a particular leg is in swing
        *
        * @param legIdx The index of the leg
        * @return True if the leg is in swing, false otherwise
        */      
        bool isLegInSwing(const short & legIdx) const;

        /**
        * @brief get the string description of the mode family
        *
        * @return The string description of the mode family
        */
        std::string description() const;

        /**
        * @brief == operator overload for comparing two mode families,
        * this calls the SteppableRegion operator== to determine if the same
        * feet are on the same regions
        *
        * @param mf1 The first mode family
        * @param mf2 The second mode family
        */
        friend bool operator==(const ModeFamily &mf1, const ModeFamily &mf2) 
        {
            for (int l = 0; l < 4; l++)
            {
                if (mf1.isLegInSwing(l) && mf2.isLegInSwing(l)) // both in swing
                    continue;
                else if (mf1.isLegInSwing(l) != mf2.isLegInSwing(l)) // one in swing and one in stance
                    return false;
                else if ( ! mf1.sameRegion(l, mf2)  ) // both in stance
                    return false;
            }
            return true; 
        };

        /**
        * @brief != operator overload for comparing two mode families,
        *
        * @param mf1 The first mode family
        * @param mf2 The second mode family
        */
        friend bool operator!=(const ModeFamily &mf1, const ModeFamily &mf2) 
        {
            return !(mf1 == mf2);
        }

        // bool closeEnoughToGoal(const vector3_t & torsoPosition) const
        // {
        //     return (torsoPosition - goalPosition_).norm() < goalRegionTolerance_;
        // }

        // bool isGoal() const { return isGoal_; }

        bool twoLegsSameRegion(const int & legIdx1, const ModeFamily &mf2, const int & legIdx2)
        {
            // std::cout << "[twoLegsSameRegion]" << std::endl;

            // std::cout << "  legIdx1: " << legIdx1 << std::endl;
            // std::cout << "  legIdx2: " << legIdx2 << std::endl;

            // std::cout << "  regions_[legIdx1].plane.positionInWorld: " << regions_[legIdx1].plane.positionInWorld.transpose() << std::endl;
            // std::cout << "  mf2.regions_[legIdx2].plane.positionInWorld: " << mf2.regions_[legIdx2].plane.positionInWorld.transpose() << std::endl;

            // std::cout << "  regions_[legIdx1].plane.orientationWorldToTerrain: " << regions_[legIdx1].plane.orientationWorldToTerrain << std::endl;
            // std::cout << "  mf2.regions_[legIdx2].plane.orientationWorldToTerrain: " << mf2.regions_[legIdx2].plane.orientationWorldToTerrain << std::endl;

            return regions_[legIdx1].plane.positionInWorld == mf2.regions_[legIdx2].plane.positionInWorld &&
                   regions_[legIdx1].plane.orientationWorldToTerrain == mf2.regions_[legIdx2].plane.orientationWorldToTerrain;
        }

    private:
        // /**
        // * @brief get the normal of the region for the given leg index in base frame
        // *
        // * @param legIdx The index of the leg
        // * @return The normal of the region for the given leg index in base frame
        // */
        // vector3_t getNormalBaseFrame(const int & legIdx) const;

        bool sameRegion(const int & legIdx, const ModeFamily &mf2) const
        {
            // Assuming that two regions will not have same transform
            return regions_[legIdx].plane.positionInWorld == mf2.regions_[legIdx].plane.positionInWorld &&
                   regions_[legIdx].plane.orientationWorldToTerrain == mf2.regions_[legIdx].plane.orientationWorldToTerrain;
        }

        // bool isGoal_ = false; /**< Whether this is the goal mode family */
        // vector3_t goalPosition_; /**< The goal position of the mode family */
        // double goalRegionTolerance_ = 0.15; /**< The tolerance for the goal region */

        rclcpp::Node::SharedPtr node_;

        ModeFamilyNodeID nodeID_;

        int nDims_ = 0; /**< The number of dimensions of the mode family */
        int oneLegDims_ = 2; /** < The number of dimensions per leg */
        int nStanceLegs_ = 0; /**< The number of stance legs in the mode family */

        std::vector<switched_model::ConvexTerrain> regions_; /**< The regions for the mode family */

};


}  // namespace mmp

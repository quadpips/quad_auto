#include <mmp_quadruped/planning/ModeFamily.h>


namespace mmp {

/****************************ModeFamily****************************/
ModeFamily::ModeFamily(const rclcpp::Node::SharedPtr& node,
                        const ModeFamilyNodeID & nodeID,
                        const switched_model::ConvexTerrain & FL_region, 
                        const switched_model::ConvexTerrain & FR_region, 
                        const switched_model::ConvexTerrain & BL_region, 
                        const switched_model::ConvexTerrain & BR_region)
{
    node_ = node;
    nodeID_ = nodeID;

    regions_.push_back(FL_region);
    regions_.push_back(FR_region);
    regions_.push_back(BL_region);
    regions_.push_back(BR_region);

    // std::vector<double> defaultCoparams(4, 0.5); // size, initial value

    int startIdxCounter = 0;
    for (int l = 0; l < 4; l++)
    {
        if (!isLegInSwing(l))
        {
            // std::vector<double> legCoparameters(oneLegDims_, 0.5);
            // allLegCoparameters_.push_back(legCoparameters);
            // nDims_ += oneLegDims_; // TODO: later, add nDims based on if it is a bar or a polygon
            nStanceLegs_++;
        } else
        {
            // std::vector<double> legCoparameters(1, -1.0);
            // allLegCoparameters_.push_back(legCoparameters); 
        }
    }
}

// ModeFamily::ModeFamily(const vector3_t & torsoLocalWaypointPosition)
// {
//     isGoal_ = true;
//     goalPosition_ = torsoLocalWaypointPosition;
// }

double ModeFamily::X(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[X] Trying to get x position of swing leg");            
    return regions_[legIdx].plane.positionInWorld.x();
}

double ModeFamily::Y(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[Y] Trying to get y position of swing leg");       
    return regions_[legIdx].plane.positionInWorld.y();
}

std::string ModeFamily::getPhase()
{
    std::string phase = "";
    if (!isLegInSwing(FL))
        phase += "LF_";

    if (!isLegInSwing(FR))
        phase += "RF_";

    if (!isLegInSwing(BL))
        phase += "LH_";

    if (!isLegInSwing(BR))
        phase += "RH";
    else
        phase.erase(phase.end() - 1);

    return phase;
}

bool ModeFamily::isFootholdOnRegionBaseFrame(const short & legIdx, const vector3_t & foothold)
{
    throw std::runtime_error("[isFootholdOnRegionBaseFrame] Not implemented for ModeFamily, use isFootholdOnRegionWorldFrame instead");
}

bool ModeFamily::isFootholdOnRegionWorldFrame(const short & legIdx, const vector3_t & foothold)
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "  [isFootholdOnRegionWorldFrame]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "    foothold: " << foothold.transpose());

    RCLCPP_INFO_STREAM(node_->get_logger(), "    region normal: " << getNormalWorldFrame(legIdx).transpose());

    scalar_t signedDistance = switched_model::terrainSignedDistanceFromPositionInWorld(foothold, regions_[legIdx].plane);

    RCLCPP_INFO_STREAM(node_->get_logger(), "    signedDistance: " << signedDistance);

    bool normalCheck = (std::abs(signedDistance) < 2 * FOOT_RADIUS);

    if (!normalCheck)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "    normalCheck failed");
        return false;
    }

    const vector3_t local_p = switched_model::positionInTerrainFrameFromPositionInWorld(foothold, regions_[legIdx].plane);
    const vector2_t local_2d_p(local_p.x(), local_p.y());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "    local_2d_p: " << local_2d_p.transpose());

    RCLCPP_INFO_STREAM(node_->get_logger(), "    regions_[legIdx].plane.positionInWorld: " << regions_[legIdx].plane.positionInWorld.transpose()); 
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    regions_[legIdx].boundary: "); 
    // for (int i = 0; i < regions_[legIdx].boundary.size(); i++)
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "      " << regions_[legIdx].boundary[i].transpose());
    
    bool planeCheck = isPointWithinConvex2dPolygonBoundary(regions_[legIdx].boundary, local_2d_p);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "    planeCheck: " << planeCheck); 

    if (!planeCheck)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "    planeCheck failed");
        return false;
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    distance2ImagePair.first: " << distance2ImagePair.first);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    distance2ImagePair.second: " << distance2ImagePair.second.transpose());

    // vector3_t local_q;
    // if (distance2ImagePair.first > 0) // if outside polygon
    //     return false;

    return true;
}

bool ModeFamily::isLegInSwing(const short & legIdx) const
{
    // if region attached to leg has no points, then it is a swing leg
    return regions_[legIdx].boundary.empty();
}

short ModeFamily::getNumDims(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[getNumDims] Trying to get num dims of swing leg");
    return oneLegDims_;
}

vector3_t ModeFamily::getNominalTorsoPosition() const
{
    bool foot_offset = false;
    bool inflated_region = false;

    vector3_t nominalContactPosition(0.0, 0.0, 0.0);
    vector3_t nominalNormal(0.0, 0.0, 0.0);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "nStanceLegs_: " << nStanceLegs_);

    if (nStanceLegs_ > 1)
    {
        for (int l = 0; l < 4; l++)
        {
            if (!isLegInSwing(l))
            {
                nominalNormal += getNormalWorldFrame(l); // TODO: real vector average
                nominalContactPosition += getNominalPositionWorldFrame(l, foot_offset, inflated_region);
            }
        }

        // not using contact normal because we only know where a subset of the feet are
        vector3_t nominalTorsoPosition = (nominalContactPosition /  nStanceLegs_) + Z_TORSO_OFFSET * (nominalNormal / nStanceLegs_); 

        return nominalTorsoPosition;
    } else
    {
        throw std::runtime_error("[getNominalTorsoPosition()]: not implemented");
    }
}

// vector3_t ModeFamily::getGoalPosition() const
// {
//     if (!isGoal_)
//         throw std::runtime_error("[getGoalPosition()]: Trying to get goal position of non-goal mode family");

//     return goalPosition_;
// }

// vector3_t ModeFamily::getMinPositionBaseFrame(const int & legIdx,
//                                                     const bool & foot_offset,
//                                                     const bool & inflated_region) const
// {
//     if (isLegInSwing(legIdx))
//         throw std::runtime_error("[getMinPosition] Trying to get min position of swing leg");        
//     return regions_[legIdx]->getMinPositionBaseFrame(foot_offset, inflated_region);
// }

// vector3_t ModeFamily::getMaxPositionBaseFrame(const int & legIdx,
//                                                     const bool & foot_offset,
//                                                     const bool & inflated_region) const
// {
//     if (isLegInSwing(legIdx))
//         throw std::runtime_error("[getMaxPosition] Trying to get max position of swing leg");        
//     return regions_[legIdx]->getMaxPositionBaseFrame(foot_offset, inflated_region);
// }

vector3_t ModeFamily::getNominalPositionWorldFrame(const int & legIdx,
                                                    const bool & foot_offset,
                                                    const bool & inflated_region) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[getNominalPosition] Trying to get nominal position of swing leg");      

    vector3_t nominalPosition = regions_[legIdx].plane.positionInWorld;
    vector3_t nominalFootOffset = foot_offset * FOOT_RADIUS * getNormalWorldFrame(legIdx); 
    // RCLCPP_INFO_STREAM(node_->get_logger(), "nominalFootOffset: " << nominalFootOffset.transpose());
    nominalPosition += nominalFootOffset;

    return nominalPosition;
}

// vector3_t ModeFamily::getNominalPositionBaseFrame(const int & legIdx,
//                                                         const bool & foot_offset,
//                                                         const bool & inflated_region) const
// {
//     if (isLegInSwing(legIdx))
//         throw std::runtime_error("[getNominalPosition] Trying to get nominal position of swing leg");        
//     return regions_[legIdx]->getNominalPositionBaseFrame(foot_offset, inflated_region);
// }

vector3_t ModeFamily::getNormalWorldFrame(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[getNormal] Trying to get normal of region for swing leg");        
    return regions_[legIdx].plane.orientationWorldToTerrain.col(2);
}

switched_model::ConvexTerrain ModeFamily::getRegion(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[getRegion] Trying to get region for swing leg");     
    return regions_[legIdx];
}

// vector3_t ModeFamily::getNormalBaseFrame(const int & legIdx) const
// {
//     if (isLegInSwing(legIdx))
//         throw std::runtime_error("[getNormal] Trying to get normal of region for swing leg");        
//     return regions_[legIdx]->getNormalBaseFrame();
// }

vector3_t ModeFamily::getRegionCoordinatesBaseFrame(const int & legIdx, 
                                                            const bool & foot_offset,
                                                            const bool & inflated_region) const
{
    throw std::runtime_error("[getRegionCoordinatesBaseFrame] Not implemented for ModeFamily, use getRegionCoordinatesWorldFrame instead");
}

vector3_t ModeFamily::getRegionCoordinatesWorldFrame(const int & legIdx, 
                                                            const bool & foot_offset,
                                                            const bool & inflated_region) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[getRegionCoordinatesBaseFrame] Trying to get stance coordinates of swing leg");    

    return getNominalPositionWorldFrame(legIdx, foot_offset, inflated_region);
}

int ModeFamily::getRegionID(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        throw std::runtime_error("[getRegionID] Trying to get region ID of swing leg");    
    else
        return nodeID_.getRegionID(legIdx);
}

std::string ModeFamily::getHashID(const int & legIdx) const
{
    if (isLegInSwing(legIdx))
        return "---";
    else
        return std::to_string(X(legIdx)) + "_" + std::to_string(Y(legIdx));
}

std::string ModeFamily::description() const
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[ModeFamily::description()]");


    // float testDouble = 0.1;
    // RCLCPP_INFO_STREAM(node_->get_logger(), std::fixed << std::setprecision(5) << testDouble); // std::setfill('0') << std::setw(4)

    // std::string testString = std::printf("%07.03f", testDouble);


    std::string description = "{";

    for (int l = 0; l < 4; l++)
    {
        if (!isLegInSwing(l))
        {
            std::stringstream xstream, ystream, regionstream;
            xstream << std::fixed << std::setprecision(3) << X(l);
            ystream << std::fixed << std::setprecision(3) << Y(l);
            regionstream << std::fixed << std::setprecision(3) << getRegionID(l);
            description += ((LegNameShort.at(l) + ": (") + ", " + regionstream.str() + ")" + (l < 3 ? ", " : "")); // xstream.str() + ", " + ystream.str() + 
        } else
        {
            description += (LegNameShort.at(l) + ": (---)"  + (l < 3 ? ", " : "")); // , ---
        }
    }
    description += "}";
    return description;
    
    // return ": {FL: (" + std::to_string(FL_region_.X()) + ", " + std::to_string(FL_region_.Y()) +
    //                     "), FR: (" + std::to_string(FR_region_.X()) + ", " + std::to_string(FR_region_.Y()) + 
    //                     "), BL: (" + std::to_string(BL_region_.X()) + ", " + std::to_string(BL_region_.Y()) +
    //                     "), BR: (" + std::to_string(BR_region_.X()) + ", " + std::to_string(BR_region_.Y()) + 
    //                     ")}"; 
}

}  // namespace mmp

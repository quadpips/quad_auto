#include <mmp_quadruped/planning/TorsoStateNode.h>


namespace mmp {


/********* TorsoTransition ********/
// const double & weight
TorsoStateNode::TorsoTransition::TorsoTransition(TorsoStateNode * nodeFrom, 
                                                 TorsoStateNode * nodeTo)
{
    // if (_nodeFrom == NULL)
    //     throw std::runtime_error("[TorsoTransition()] _nodeFrom is null");

    // if (_nodeTo == NULL)
    //     throw std::runtime_error("[TorsoTransition()] _nodeTo is null");

    nodeFrom_ = nodeFrom;
    nodeTo_ = nodeTo;

    weight_ = nodeFrom_->calculateDistance(nodeTo_);
}

// void TorsoStateNode::TorsoTransition::transformToBaseFrame(const geometry_msgs::msg::TransformStamped & worldFrameToBaseFrameTransform)
// {
//     nodeFrom_->transformToBaseFrame(worldFrameToBaseFrameTransform);
//     nodeTo_->transformToBaseFrame(worldFrameToBaseFrameTransform);

//     return;
// }

/********* TorsoStateNode ********/
TorsoStateNode::TorsoStateNode(const vector3_t & torsoPositionWorldFrame, 
                                const vector3_t & torsoOrientation, 
                                const vector3_t & goalPositionWorldFrame,
                                const vector3_t & goalOrientation)
{
    // std::cout << "[TorsoStateNode():]" << std::endl;
    // this->positionWorldFrame = torsoPositionWorldFrame; // world frame
    // this->positionBaseFrame = vector3_t(0.0, 0.0, 0.0);
    // this->theta = theta;
    // this->orientationWorldFrame = torsoOrientation;
    this->poseWorldFrame.head(3) = torsoOrientation; // TODO: robustify this
    this->poseWorldFrame.tail(3) = torsoPositionWorldFrame; // TODO: robustify this

    this->costToCome_ = 1e30;
    this->costToGo_ = calculateCostToGo(goalPositionWorldFrame, goalOrientation);

    // std::cout << "v_idx: " << v_idx << ", h_idx: " << h_idx << std::endl;
    // std::cout << "x: " << x << ", y: " << y << ", goalCoMX: " << goalCoMX << ", " << goalCoMY << std::endl;

    // this->goal = goal;

    // if (this->goal) std::cout << "making goal node" << std::endl;

    this->visited = false;
    this->explored = false;
}

TorsoStateNode::~TorsoStateNode()
{
    for (auto &tran : trans)
        delete tran;
}

double TorsoStateNode::calculatePositionalDistance(TorsoStateNode * other)
{
    vector3_t positionWorldFrame = extractTorsoPosition(poseWorldFrame);
    vector3_t otherPositionWorldFrame = extractTorsoPosition(other->poseWorldFrame);
    return (positionWorldFrame - otherPositionWorldFrame).norm();
}

double TorsoStateNode::calculateOrientationDistance(TorsoStateNode * other)
{
    vector3_t orientationWorldFrame = extractTorsoOrientation(poseWorldFrame);
    vector3_t otherOrientationWorldFrame = extractTorsoOrientation(other->poseWorldFrame);
    Eigen::Vector2d torsoHeading(cos(orientationWorldFrame[2]), sin(orientationWorldFrame[2]));
    Eigen::Vector2d goalHeading(cos(otherOrientationWorldFrame[2]), sin(otherOrientationWorldFrame[2]));
    double ori_ang_diff = std::acos( torsoHeading.dot(goalHeading) / (torsoHeading.norm() * goalHeading.norm()) );
    return std::abs(ori_ang_diff);
}

double TorsoStateNode::calculateDistance(TorsoStateNode * other)
{
    double pos_dist = calculatePositionalDistance(other);
    double ori_dist = calculateOrientationDistance(other);
    return pos_dist + ori_dist; // weight two terms?
}

// void TorsoStateNode::transformToBaseFrame(const geometry_msgs::msg::TransformStamped & worldFrameToBaseFrameTransform)
// {
//     positionBaseFrame = transformHelperPointStamped(positionWorldFrame, worldFrameToBaseFrameTransform);
//     return;
// }

double TorsoStateNode::calculateCostToGo(const vector3_t & goalPositionWorldFrame,
                                            const vector3_t & goalOrientationWorldFrame)
{
    vector3_t positionWorldFrame = extractTorsoPosition(poseWorldFrame);
    double pos_dist = (positionWorldFrame - goalPositionWorldFrame).norm();

    vector3_t orientationWorldFrame = extractTorsoOrientation(poseWorldFrame);
    Eigen::Vector2d torsoHeading(cos(orientationWorldFrame[2]), sin(orientationWorldFrame[2]));
    Eigen::Vector2d goalHeading(cos(goalOrientationWorldFrame[2]), sin(goalOrientationWorldFrame[2]));
    double ori_ang_diff = std::acos( torsoHeading.dot(goalHeading) / (torsoHeading.norm() * goalHeading.norm()) );
    double ori_dist = std::abs(ori_ang_diff);
    return pos_dist + ori_dist; // weight two terms?
}

void TorsoStateNode::addTransition(TorsoStateNode * dst_node)
{
    TorsoStateNode::TorsoTransition * tran = new TorsoStateNode::TorsoTransition(this, dst_node);

    trans.push_back(tran);

    return;
}

std::string TorsoStateNode::description()
{
    vector3_t positionWorldFrame = extractTorsoPosition(poseWorldFrame);
    vector3_t orientationWorldFrame = extractTorsoOrientation(poseWorldFrame);

    std::string desc = "{x: " + std::to_string(positionWorldFrame[0]) + 
                      ", y: " + std::to_string(positionWorldFrame[1]) + 
                      ", z: " + std::to_string(positionWorldFrame[2]) + "}" + 
                      ", {roll: " + std::to_string(orientationWorldFrame[0]) + 
                      ", pitch: " + std::to_string(orientationWorldFrame[1]) + 
                      ", yaw: " + std::to_string(orientationWorldFrame[2]) + "}";
    return desc;
}

}  // namespace mmp

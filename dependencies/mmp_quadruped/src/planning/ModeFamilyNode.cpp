#include <mmp_quadruped/planning/ModeFamilyNode.h>


namespace mmp {

/*********************************ModeFamilyNode*****************************/
ModeFamilyNode::Transition::Transition(ModeFamilyNode * nFrom, ModeFamilyNode * nTo, const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                                    const double & w_exp, const double & w_euc, const double & w_stance, const double & w_step, const double & w_torso)
{
    // std::cout << "[Transition()]" << std::endl;
    nodeFrom = nFrom;
    nodeTo = nTo;
    traversalCount = 0;

    nDims_ = nodeFrom->modeFamily->getNumDims() + nodeTo->modeFamily->getNumDims();

    this->w_exp = w_exp;
    this->w_euc = w_euc;
    this->w_stance = w_stance;
    this->w_step = w_step;
    this->w_torso = w_torso;

    this->weight = 0.0001; // + (w_FL + w_FR + w_BL + w_BR);

    // this->weight = 0.0001 + (w_FL_first + w_FL_second + w_FR_first + w_FR_second + 
    //                 w_BL_first + w_BL_second + w_BR_first + w_BR_second);

    // if (this->weight > 0.0001)
    // {
    //     std::cout << "  higher cost" << std::endl;
    //     std::cout << "  nodeFrom: " << nodeFrom->description(false) << std::endl;
    //     std::cout << "  nodeTo: " << nodeTo->description(false) << std::endl;
    //     std::cout << "    w_FL_first: " << w_FL_first << ", w_FL_second: " << w_FL_second << ", " <<
    //                  "    w_FR_first: " << w_FR_first << ", w_FR_second: " << w_FR_second << ", " << 
    //                  "    w_BL_first: " << w_BL_first << ", w_BL_second: " << w_BL_second << ", " << 
    //                  "    w_BR_first: " << w_BR_first << ", w_FR_second: " << w_BR_second << std::endl;
    // }

    // torsoPathDeviation = calculateTorsoPathDeviation(torsoPath);
    stanceDeviation_ = calculateStanceDeviationWeight();
    consistencyDeviation_ = calculateConsistencyDeviationWeight(); 
}

ModeFamilyNode::Transition::Transition(ModeFamilyNode * nFrom, ModeFamilyNode * nTo)
{
    nodeFrom = nFrom;
    nodeTo = nTo;
    traversalCount = 0;

    nDims_ = 4;
}

ModeFamilyNode::Transition::~Transition()
{
}

// double ModeFamilyNode::Transition::calculateTorsoPathDeviation(const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath)
// {
//     // std::cout << "[calculateTorsoPathDeviation()]" << std::endl;
//     double min_dist = 1e30;

//     vector3_t nominalTorsoPosition = calculateNominalTorsoPositionWorldFrame();
//     // vector3_t nominalTorsoPosition = calculateNominalTorsoPositionBaseFrame();

//     // vector3_t fromNominalTorsoPosition = nodeFrom->calculateNominalTorsoPositionBaseFrame();
//     // vector3_t toNominalTorsoPosition = nodeTo->calculateNominalTorsoPositionBaseFrame();

//     // std::cout << "  nominalTorsoPosition: " << nominalTorsoPosition.transpose() << std::endl;
//     // std::cout << "  toNominalTorsoPosition: " << toNominalTorsoPosition.transpose() << std::endl;

//     // std::cout << "entering loop" << std::endl;
//     for (TorsoStateNode::TorsoTransition * tran : torsoPath)
//     {

//         if (tran->getNodeFrom() == NULL)
//         {
//             // std::cout << "From node is null" << std::endl;
//             throw std::runtime_error("From node is null");
//         }

//         if (tran->getNodeTo() == NULL)
//         {
//             // std::cout << "To node is null" << std::endl;
//             throw std::runtime_error("To node is null");   
//         }
//         // std::cout << "      torso pose from: (" << tran->getNodeFrom()->X() << ", " << tran->getNodeFrom()->Y() << ")" << std::endl; 
//         // std::cout << "      torso pose to: (" << tran->getNodeTo()->X() << ", " << tran->getNodeTo()->Y() << ")" << std::endl; 

//         // std::cout << "calculating fromDist" << std::endl;
//         double fromDist = (nominalTorsoPosition - tran->getNodeFrom()->getPositionWorldFrame()).norm();
//         // std::cout << "          fromDist: " << fromDist << std::endl;
//         double toDist = (nominalTorsoPosition - tran->getNodeTo()->getPositionWorldFrame()).norm();
//         // std::cout << "          toDist: " << toDist << std::endl;

//         double avg_dist = (fromDist + toDist) / 2.0;
//         // std::cout << "      avg_dist: " << avg_dist << std::endl;
//         if (avg_dist < min_dist)
//             min_dist = avg_dist;
//     }
//     // std::cout << "finished calculateTorsoPathDeviation" << std::endl;

//     // std::cout << "  min_dist: " << min_dist << std::endl;

//     torsoPathDeviation = min_dist;
//     return min_dist;
// }

vector3_t ModeFamilyNode::Transition::calculateNominalTorsoPositionWorldFrame() const
{


    // get nominal positions of the four regions
    vector3_t nominalFLPosition = nodeFrom->modeFamily->isLegInSwing(FL) ? nodeTo->modeFamily->getNominalPositionWorldFrame(FL) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(FL);       
    vector3_t nominalFRPosition = nodeFrom->modeFamily->isLegInSwing(FR) ? nodeTo->modeFamily->getNominalPositionWorldFrame(FR) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(FR);
    vector3_t nominalBLPosition = nodeFrom->modeFamily->isLegInSwing(BL) ? nodeTo->modeFamily->getNominalPositionWorldFrame(BL) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(BL);       
    vector3_t nominalBRPosition = nodeFrom->modeFamily->isLegInSwing(BR) ? nodeTo->modeFamily->getNominalPositionWorldFrame(BR) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(BR);    
    
    // std::cout << "      nominalFLPosition: " << nominalFLPosition.transpose() << std::endl;
    // std::cout << "      nominalFRPosition: " << nominalFRPosition.transpose() << std::endl;
    // std::cout << "      nominalBLPosition: " << nominalBLPosition.transpose() << std::endl;
    // std::cout << "      nominalBRPosition: " << nominalBRPosition.transpose() << std::endl;

    // average four feet to get CoM position of thisNode
    vector3_t copPosition = (nominalFLPosition + nominalFRPosition + nominalBLPosition + nominalBRPosition) / 4.0;

    // vector3_t contactVector1 = nominalFRPosition - nominalBRPosition;
    // vector3_t contactVector2 = nominalBLPosition - nominalBRPosition;
    // vector3_t localContactNormal = contactVector1.cross(contactVector2).normalized();    
    
    std::vector<vector3_t> nominalPositions = {nominalFLPosition, nominalFRPosition, nominalBLPosition, nominalBRPosition};
    vector3_t localContactNormal = estimatePlaneNormal(nominalPositions);

    vector3_t nominalTorsoPosition = copPosition + localContactNormal * Z_TORSO_OFFSET;

    // nominalTorsoPosition[2] += Z_TORSO_OFFSET;

    return nominalTorsoPosition;
}

// vector3_t ModeFamilyNode::Transition::calculateNominalTorsoPositionBaseFrame() const
// {
//     // get nominal positions of the four regions
//     vector3_t nominalFLPosition = nodeFrom->modeFamily->isLegInSwing(FL) ? nodeTo->modeFamily->getNominalPositionBaseFrame(FL) :
//                                                                                  nodeFrom->modeFamily->getNominalPositionBaseFrame(FL);       
//     vector3_t nominalFRPosition = nodeFrom->modeFamily->isLegInSwing(FR) ? nodeTo->modeFamily->getNominalPositionBaseFrame(FR) :
//                                                                                  nodeFrom->modeFamily->getNominalPositionBaseFrame(FR);
//     vector3_t nominalBLPosition = nodeFrom->modeFamily->isLegInSwing(BL) ? nodeTo->modeFamily->getNominalPositionBaseFrame(BL) :
//                                                                                  nodeFrom->modeFamily->getNominalPositionBaseFrame(BL);       
//     vector3_t nominalBRPosition = nodeFrom->modeFamily->isLegInSwing(BR) ? nodeTo->modeFamily->getNominalPositionBaseFrame(BR) :
//                                                                                  nodeFrom->modeFamily->getNominalPositionBaseFrame(BR);    
    
//     // std::cout << "      nominalFLPosition: " << nominalFLPosition.transpose() << std::endl;
//     // std::cout << "      nominalFRPosition: " << nominalFRPosition.transpose() << std::endl;
//     // std::cout << "      nominalBLPosition: " << nominalBLPosition.transpose() << std::endl;
//     // std::cout << "      nominalBRPosition: " << nominalBRPosition.transpose() << std::endl;

//     // average four feet to get CoM position of thisNode
//     vector3_t copPosition = (nominalFLPosition + nominalFRPosition + nominalBLPosition + nominalBRPosition) / 4.0;

//     // vector3_t contactVector1 = nominalFRPosition - nominalBRPosition;
//     // vector3_t contactVector2 = nominalBLPosition - nominalBRPosition;
//     // vector3_t localContactNormal = contactVector1.cross(contactVector2).normalized();    
    
//     std::vector<vector3_t> nominalPositions = {nominalFLPosition, nominalFRPosition, nominalBLPosition, nominalBRPosition};
//     vector3_t localContactNormal = estimatePlaneNormal(nominalPositions);

//     vector3_t nominalTorsoPosition = copPosition + localContactNormal * Z_TORSO_OFFSET;

//     // nominalTorsoPosition[2] += Z_TORSO_OFFSET;

//     return nominalTorsoPosition;
// }

double ModeFamilyNode::Transition::calculateEuclideanWeight()
{
    return nodeFrom->calculateEuclideanDistance(*nodeTo);
}

double ModeFamilyNode::Transition::getEuclideanWeightTerm()
{
    return w_euc * calculateEuclideanWeight();
}

double ModeFamilyNode::Transition::getModeFamilyWeight()
{
    return weight;
}

double ModeFamilyNode::Transition::getModeFamilyWeightTerm()
{
    return w_exp * getModeFamilyWeight();
}

double ModeFamilyNode::Transition::getTorsoPathDeviationWeight() 
{ 
    return torsoPathDeviation; 
}

double ModeFamilyNode::Transition::getTorsoPathDeviationWeightTerm()
{
    return w_torso * getTorsoPathDeviationWeight();
}

// double ModeFamilyNode::Transition::getSteppabilityWeight(Camera * camera)
// {
//     double steppability_score = 0.0;


//     if (camera)
//     {
//         for (int l = 0; l < 4; l++)
//         {
//             if (nodeFrom->modeFamily->isLegInSwing(l))
//             {
//                 double foothold_score = camera->multiPointSteppabilityScoreQuery(nodeTo->modeFamily->getNominalPositionWorldFrame(l));
//                 steppability_score += foothold_score;
//             }
//         }
//     }

//     return steppability_score;
// }

// double ModeFamilyNode::Transition::getSteppabilityWeight(Camera * camera, const std::vector<std::vector<short>> & tranWeightMatrixInds)
// {
//     // std::cout << "      [getSteppabilityWeight()]" << std::endl;

//     double steppability_score = 0.0;

//     bool foot_offset = false;
//     bool inflated_region = true;

//     if (camera) // if camera exists
//     {
//         for (int l = 0; l < 4; l++) // for four legs
//         {
//             if (nodeFrom->modeFamily->isLegInSwing(l)) // if leg is swinging now and will be in contact at NodeTo
//             {
//                 // std::cout << "          leg: " << l << std::endl;
//                 std::vector<double> coparams = std::vector<double>(tranWeightMatrixInds.at(l).size());
//                 for (int i = 0; i < coparams.size(); i++)
//                 {
//                     coparams.at(i) = tranWeightMatrixInds.at(l).at(i) * ModeFamilyNode::Transition::expStep;
//                     if (coparams.at(i) < 0.0 || coparams.at(i) > 1.0)
//                         throw std::runtime_error("coparams value out of bounds: " + std::to_string(coparams.at(i)));
//                 }

//                 // std::cout << "          coparams: " << vec2string(coparams) << std::endl;

//                 vector3_t destination_point = nodeTo->modeFamily->getRegionCoordinatesBaseFrame(l, coparams, foot_offset, inflated_region);

//                 // std::cout << "          foothold: " << nodeTo->modeFamily->getRegionCoordinatesBaseFrame(l, coparams, false).transpose() << std::endl;

//                 double foothold_score = camera->multiPointSteppabilityScoreQuery(destination_point);
//                 // // std::cout << "          foothold_score: " << foothold_score << std::endl;

//                 steppability_score += foothold_score;
//             }
//         }
//     }

//     return steppability_score;
// }

// int ModeFamilyNode::Transition::getSteppabilityLabel(Camera * camera)
// {
//     int steppability_label = -1;


//     if (camera)
//     {
//         for (int l = 0; l < 4; l++)
//         {
//             if (nodeFrom->modeFamily->isLegInSwing(l))
//             {
//                 steppability_label = camera->steppabilityLabelQuery(nodeTo->modeFamily->getNominalPositionWorldFrame(l));
//                 if (steppability_label < PASS)
//                     break;
//             }
//         }
//     }

//     return steppability_label;
// }

// double ModeFamilyNode::Transition::getSteppabilityWeightTerm(Camera * camera)
// {
//     return w_step * getSteppabilityWeight(camera);
// }

// double ModeFamilyNode::Transition::getSteppabilityWeightTerm(Camera * camera, const std::vector<std::vector<short>> & tranWeightMatrixInds)
// {
//     return w_step * getSteppabilityWeight(camera, tranWeightMatrixInds);
// }

double ModeFamilyNode::Transition::getStanceDeviationWeight()
{
    return stanceDeviation_;
}



double ModeFamilyNode::Transition::getStanceDeviationWeightTerm()
{
    return w_stance * getStanceDeviationWeight();
}

double ModeFamilyNode::Transition::calculateConsistencyDeviationWeight()
{
    if (nodeFrom->previousNode == NULL)
        return 0.0;

    // Designed to encourage planner to keep a consistent stepping pattern 

    // get nominal stance of thisNode
    vector3_t previousTorsoNominalTorsoPosition = nodeFrom->previousNode->modeFamily->getNominalTorsoPosition();
    vector3_t toTorsoNominalTorsoPosition = nodeTo->modeFamily->getNominalTorsoPosition();
    
    double consistencyDeviation = 0.0;

    for (int l = 0; l < 4; l++)
    {
        if (nodeFrom->modeFamily->isLegInSwing(l))
        {
            vector3_t previousNominalPosition = nodeFrom->previousNode->modeFamily->getNominalPositionWorldFrame(l);
            vector3_t fromOffset = previousNominalPosition - previousTorsoNominalTorsoPosition;

            vector3_t toNominalPosition = nodeTo->modeFamily->getNominalPositionWorldFrame(l);
            vector3_t toOffset = toNominalPosition - toTorsoNominalTorsoPosition;

            double dist = (fromOffset - toOffset).norm();
            consistencyDeviation += dist;
        }
    }

    return consistencyDeviation;
}

double ModeFamilyNode::Transition::getConsistencyDeviationWeight()
{
    return consistencyDeviation_;
}

double ModeFamilyNode::Transition::getConsistencyDeviationWeightTerm()
{
    return w_consistency * getConsistencyDeviationWeight();
}


double ModeFamilyNode::Transition::calculateStanceDeviationWeight()
{
    // std::cout << "[getStanceDeviationWeight()]" << std::endl;


    // std::cout << "nodeFrom descr: " << nodeFrom->modeFamily->description() << std::endl;
    // std::cout << "nodeTo descr: " << nodeTo->modeFamily->description() << std::endl;

    // get nominal positions of the four regions
    vector3_t nominalFLPosition = nodeFrom->modeFamily->isLegInSwing(FL) ? nodeTo->modeFamily->getNominalPositionWorldFrame(FL) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(FL);       
    vector3_t nominalFRPosition = nodeFrom->modeFamily->isLegInSwing(FR) ? nodeTo->modeFamily->getNominalPositionWorldFrame(FR) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(FR);
    vector3_t nominalBLPosition = nodeFrom->modeFamily->isLegInSwing(BL) ? nodeTo->modeFamily->getNominalPositionWorldFrame(BL) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(BL);       
    vector3_t nominalBRPosition = nodeFrom->modeFamily->isLegInSwing(BR) ? nodeTo->modeFamily->getNominalPositionWorldFrame(BR) :
                                                                                 nodeFrom->modeFamily->getNominalPositionWorldFrame(BR);    

    // std::cout << "      nominalFLPosition: " << nominalFLPosition.transpose() << std::endl;
    // std::cout << "      nominalFRPosition: " << nominalFRPosition.transpose() << std::endl;
    // std::cout << "      nominalBLPosition: " << nominalBLPosition.transpose() << std::endl;
    // std::cout << "      nominalBRPosition: " << nominalBRPosition.transpose() << std::endl;

    double idealStanceWidth = 0.25;
    double idealStanceLength = 0.40;
    // double idealStanceDiagonal = std::sqrt( std::pow(idealStanceWidth, 2.0) + std::pow(idealStanceLength, 2.0));

    vector3_t BLtoBR = nominalBRPosition - nominalBLPosition;
    vector3_t BRtoFR = nominalFRPosition - nominalBRPosition;
    vector3_t FRtoFL = nominalFLPosition - nominalFRPosition;
    vector3_t FLtoBL = nominalBLPosition - nominalFLPosition;

    double distDeviationBLtoBR = std::abs(idealStanceWidth - BLtoBR.norm());
    double distDeviationBRtoFR = std::abs(idealStanceLength - BRtoFR.norm());
    double distDeviationFRtoFL = std::abs(idealStanceWidth - FRtoFL.norm());
    double distDeviationFLtoBL = std::abs(idealStanceLength - FLtoBL.norm());

    // std::cout << "      distDeviationBLtoBR: " << distDeviationBLtoBR << std::endl;
    // std::cout << "      distDeviationBRtoFR: " << distDeviationBRtoFR << std::endl;
    // std::cout << "      distDeviationFRtoFL: " << distDeviationFRtoFL << std::endl;
    // std::cout << "      distDeviationFLtoBL: " << distDeviationFLtoBL << std::endl;

    double distPenalty = std::exp(distDeviationBLtoBR) + std::exp(distDeviationBRtoFR) + std::exp(distDeviationFRtoFL) + std::exp(distDeviationFLtoBL);

    // std::cout << "      distPenalty: " << distPenalty << std::endl;

    double idealStanceAngle = M_PI / 2;
    double angDeviationBL = std::abs(idealStanceAngle - std::abs(vec2vecAngularDiff(BLtoBR, FLtoBL)));
    double angDeviationBR =  std::abs(idealStanceAngle - std::abs(vec2vecAngularDiff(BRtoFR, BLtoBR)));
    double angDeviationFR = std::abs(idealStanceAngle - std::abs(vec2vecAngularDiff(FRtoFL, BRtoFR)));
    double angDeviationFL = std::abs(idealStanceAngle - std::abs(vec2vecAngularDiff(FLtoBL, FRtoFL)));
    double angDeviation = angDeviationBL + angDeviationBR + angDeviationFR + angDeviationFL;

    // std::cout << "      angDeviationBL: " << angDeviationBL << std::endl;
    // std::cout << "      angDeviationBR: " << angDeviationBR << std::endl;
    // std::cout << "      angDeviationFR: " << angDeviationFR << std::endl;
    // std::cout << "      angDeviationFL: " << angDeviationFL << std::endl;

    // std::cout << "      angDeviation: " << angDeviation << std::endl;

    double stanceDeviation = distPenalty + angDeviation; 

    // if (stanceDeviation < 0.001)
    // {
    //     std::cout << "      nominalFLPosition: " << nominalFLPosition.transpose() << std::endl;
    //     std::cout << "      nominalFRPosition: " << nominalFRPosition.transpose() << std::endl;
    //     std::cout << "      nominalBLPosition: " << nominalBLPosition.transpose() << std::endl;
    //     std::cout << "      nominalBRPosition: " << nominalBRPosition.transpose() << std::endl;

    //     std::cout << "      stanceDeviation: " << stanceDeviation << std::endl;
    // }

    return stanceDeviation;                                                                                                                                                     
}

void ModeFamilyNode::Transition::updateWeightWithTrajOptCost(const double & trajopt_cost)
{
    // std::cout << "      [updateWeightWithTrajOptCost()]:" << std::endl;
    // double processed_cost = tanh(trajopt_cost);

    // double new_weight = weight + processed_cost;
    // std::cout << "  weight: " << weight << std::endl; 
    // std::cout << " trajopt cost: " << trajopt_cost << std::endl;
    // double new_weight = weight + trajopt_cost;

    // std::cout << "          prior traversal count: " << (traversalCount - 1) << std::endl;
    // std::cout << "          prior weight: " << weight << std::endl;
    // std::cout << "          new evaluation: " << trajopt_cost << std::endl;
    // std::cout << "          new traversal count: " << traversalCount << std::endl;

    double new_average_evaluation = ((traversalCount - 1) * weight + trajopt_cost) / (traversalCount);

    // std::cout << "          new weight: " << new_average_evaluation << std::endl;

    weight = new_average_evaluation;
}

std::string ModeFamilyNode::Transition::description(const bool & includeTransitions)
{
    return "[Transition]:\n  FROM: " + nodeFrom->description(includeTransitions) + " \n  TO: " + nodeTo->description(includeTransitions);
}    

/*-----------------*/
ModeFamilyNode::ModeFamilyNode(ModeFamily &mf, const base_coordinate_t & refTorsoPose, const short & search_algorithm)
{
    this->modeFamily = &mf;
    this->search_algorithm = search_algorithm;
    this->refTorsoPose = refTorsoPose;

    this->explored = false;
    this->costToCome_ = 1e30;
}

ModeFamilyNode::~ModeFamilyNode()
{
    delete modeFamily;
    // delete previousNode;
    // delete previousTransition;

    for (ModeFamilyNode::Transition * tran : trans)
        delete tran;

    // trans.clear();
}

void ModeFamilyNode::resetNode()
{
    // for mode family search
    explored = false;
    visited = false;
    costToCome_ = 1e30;

}

// used for non-goal nodes
double ModeFamilyNode::calculateEuclideanDistance(const ModeFamilyNode & otherNode)
{
    // std::cout << "[ModeFamilyNode::calculateEuclideanDistance() to other node]" << std::endl;
    // get nominal stance of thisNode
    vector3_t thisNominalTorsoPosition = this->modeFamily->getNominalTorsoPosition();
    // std::cout << "  thisNominalTorsoPosition: " << thisNominalTorsoPosition.transpose() << std::endl;
    
    // get nominal stance of otherNode
    vector3_t otherNominalTorsoPosition = otherNode.modeFamily->getNominalTorsoPosition();
    // std::cout << "  otherNominalTorsoPosition: " << otherNominalTorsoPosition.transpose() << std::endl;

    // calculate and return Euclidean distance 
    double posn_dist = (thisNominalTorsoPosition - otherNominalTorsoPosition).norm();
    // std::cout << "  posn_dist: " << posn_dist << std::endl;

    vector3_t otherOrientation = extractTorsoOrientation(otherNode.getRefTorsoPose());
    quaternion_t otherQuat = quaternionFromEulerXYZ(otherOrientation);
    // std::cout << "  otherOrientation: " << otherOrientation.transpose() << std::endl;
    // std::cout << "  otherQuat: " << otherQuat.coeffs().transpose() << std::endl;

    vector3_t thisOrientation = extractTorsoOrientation(this->refTorsoPose);
    quaternion_t thisQuat = quaternionFromEulerXYZ(thisOrientation);
    // std::cout << "  thisOrientation: " << thisOrientation.transpose() << std::endl;
    // std::cout << "  thisQuat: " << thisQuat.coeffs().transpose() << std::endl;

    double ang_dist = quaternionAngularDiff(thisQuat, otherQuat);
    // std::cout << "  ang_dist: " << ang_dist << std::endl;

    double dist = posn_dist + 100.0 * ang_dist;    
    // std::cout << "  total dist: " << dist << std::endl;
    
    return dist;
}

// only used for goal node
double ModeFamilyNode::calculateEuclideanDistance(const base_coordinate_t & goalPose)
{
    // std::cout << "[ModeFamilyNode::calculateEuclideanDistance() to goal node]" << std::endl;

    vector3_t goalPosition = extractTorsoPosition(goalPose);
    // std::cout << "  goalPosition: " << goalPosition.transpose() << std::endl;

    // get nominal stance of thisNode
    vector3_t thisNominalTorsoPosition = this->modeFamily->getNominalTorsoPosition();
    // std::cout << "  thisNominalTorsoPosition: " << thisNominalTorsoPosition.transpose() << std::endl;

    // calculate and return Euclidean distance 
    double posn_dist = (thisNominalTorsoPosition - goalPosition).norm();
    // std::cout << "  posn_dist: " << posn_dist << std::endl;
    
    vector3_t goalOrientation = extractTorsoOrientation(goalPose);
    quaternion_t goalQuat = quaternionFromEulerXYZ(goalOrientation);
    // std::cout << "  goalOrientation: " << goalOrientation.transpose() << std::endl;
    // std::cout << "  goalQuat: " << goalQuat.coeffs().transpose() << std::endl;

    vector3_t thisOrientation = extractTorsoOrientation(this->refTorsoPose);
    quaternion_t thisQuat = quaternionFromEulerXYZ(thisOrientation);
    // std::cout << "  thisOrientation: " << thisOrientation.transpose() << std::endl;
    // std::cout << "  thisQuat: " << thisQuat.coeffs().transpose() << std::endl;

    double ang_dist = quaternionAngularDiff(thisQuat, goalQuat);
    // std::cout << "  ang_dist: " << ang_dist << std::endl;

    double dist = posn_dist + 100.0 * ang_dist;
    // std::cout << "  total dist: " << dist << std::endl;
    
    return dist;
}

std::string ModeFamilyNode::description(const bool & includeTransitions)
{
    // std::cout << "[ModeFamilyNode::description()]" << std::endl;
    // double c = getCost();
    std::string str = modeFamily->description(); // + ", node cost: " + std::to_string(cost);
    
    // std::cout << "  1" << std::endl;

    // std::cout << "  2" << std::endl;

    if (includeTransitions)
    {
        for (ModeFamilyNode::Transition * t : trans)
        {
            str += "\n  [transition to node]: {" + t->getNodeTo()->modeFamily->description() + "}"; // ", node cost: " + std::to_string(t->getNodeTo()->getCost()) +
        }
    }

    // std::cout << "  3" << std::endl;
    
    return str;
}

void ModeFamilyNode::addTransitionToGoal(ModeFamilyNode * nt)
{
    trans.push_back(new Transition(this, nt));
}

void ModeFamilyNode::addTransitionTo(ModeFamilyNode * nt, const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                                const double & w_exp, const double & w_euc, const double & w_stance, const double & w_step, const double & w_torso)
{
    trans.push_back(new Transition(this, nt, torsoPath, w_exp, w_euc, w_stance, w_step, w_torso));
}

}  // namespace mmp
 
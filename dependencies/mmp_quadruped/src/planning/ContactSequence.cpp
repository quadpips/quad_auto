#include <mmp_quadruped/planning/ContactSequence.h>


namespace mmp {

/****************************ContactSequence****************************/
ContactSequence::ContactSequence(const std::string & gaitCommandFile, const std::string & gait)
{
    bool verbose = true;

    if (gait != "standing_trot" && gait != "slow_trot" && gait != "standing_walk")
    {
        throw std::runtime_error("[ContactSequence] gait: " + gait + " is not supported");
    }


    // parse gaitCommandFile and load gait information

    /////////////////////////////////////////////////
    ///////////////// MODE SEQUENCE ///////////////// 
    /////////////////////////////////////////////////

    ocs2::loadData::loadStdVector(gaitCommandFile, gait + ".modeSequence", modeSequenceString_, verbose);

    if (modeSequenceString_.empty())
        throw std::runtime_error("[ContactSequence] failed to load mode sequence: " + gait + " from " + gaitCommandFile);

    // std::cout << "Mode sequence: " << std::endl;
    for (int i = 0; i < modeSequenceString_.size(); i++)
    {
        std::string modeString = modeSequenceString_[i];
        // std::cout << "      " << modeString << std::endl;      

        if (modeString != "STANCE")
            numSwingPhases++;
    }

    ocs2::loadData::loadStdVector(gaitCommandFile, gait + ".switchingTimes", switchingTimesString_, verbose);

    if (switchingTimesString_.empty())
        throw std::runtime_error("[ContactSequence] failed to load switching times: " + gait + " from " + gaitCommandFile);

    // std::cout << "Switching Times: " << std::endl;
    for (int i = 0; i < switchingTimesString_.size(); i++)
    {
        std::string switchingTimeString = switchingTimesString_[i];
        // std::cout << "      " << switchingTimeString << std::endl;      
    }


    // We assume that the gait in use has the following format:
    /*
    * standing_pace
    * {
    *      modeSequence
    *      {
    *          [0]     SWING (any combination)
    *          [1]     STANCE
    *          [2]     SWING (any combination)
    *          [3]     STANCE
    *           .
    *           .   
    *           .
    *      }
    *      switchingTimes
    *      {
    *          [0]     0.0
    *          [1]     swingTime
    *          [2]     swingTime + stanceTime
    *          [3]     2 * swingTime + stanceTime
    *          [4]     2 * swingTime + 2 * stanceTime
    *           .
    *           .
    *           .
    *      }
    * }
    */

    // std::cout << "modeSequenceString_: " << std::endl;
    for (int i = 0; i < (modeSequenceString_.size() - 1); i++)
    {
        // Check if the mode sequence is valid
        if ((modeSequenceString_[i] == "STANCE" && modeSequenceString_[i + 1] != "STANCE") || 
            (modeSequenceString_[i] != "STANCE" && modeSequenceString_[i + 1] == "STANCE"))
            continue;
        else
            throw std::runtime_error("[ContactSequence] mode sequence: " + gait + " from " + gaitCommandFile + " is not valid");
    }

    for (int i = 0; i < (modeSequenceString_.size() - 1); i++)
    {
        // Calculate swing and stance times and check if they are valid
        if (modeSequenceString_[i] != "STANCE")
        {

            if (swingTime < 0.0)
            {
                swingTime = std::stod(switchingTimesString_[i + 1]) - std::stod(switchingTimesString_[i]);
            } else
            {
                double otherSwingTime = std::stod(switchingTimesString_[i + 1]) - std::stod(switchingTimesString_[i]); 
                if (std::abs(otherSwingTime - swingTime) > 1e-6)
                    throw std::runtime_error("[ContactSequence] swing time is not consistent, otherSwingTime: " + std::to_string(otherSwingTime) + ", swingTime: " + std::to_string(swingTime));
            }
        } else
        {
            if (stanceTime < 0.0)
            {
                stanceTime = std::stod(switchingTimesString_[i + 1]) - std::stod(switchingTimesString_[i]);
            } else
            {
                double otherStanceTime = std::stod(switchingTimesString_[i + 1]) - std::stod(switchingTimesString_[i]); 
                if (std::abs(otherStanceTime - stanceTime) > 1e-6)
                    throw std::runtime_error("[ContactSequence] stance time is not consistent, otherStanceTime: " + std::to_string(otherStanceTime) + ", stanceTime: " + std::to_string(stanceTime));
            }
        }  
    }

    ///////////////////////////////////////////////////
    ///////////////// SWITCHING TIMES ///////////////// 
    ///////////////////////////////////////////////////

    // std::cout << "numSwingPhases: " << numSwingPhases << std::endl;

    if (numSwingPhases == 0)
        throw std::runtime_error("[ContactSequence] number of swing phases is not correct: " + std::to_string(numSwingPhases));

    // std::cout << "swingTime: " << swingTime << std::endl;

    if (swingTime < 0.0)
        throw std::runtime_error("[ContactSequence] swing time is not correct: " + std::to_string(swingTime));

    // std::cout << "stanceTime: " << stanceTime << std::endl;

    if (stanceTime < 0.0)
        throw std::runtime_error("[ContactSequence] stance time is not correct:" + std::to_string(stanceTime));

    swingPhaseCounter_ = 0;
}

bool ContactSequence::doPhasesFollow(const std::string & beforePhase, const std::string & afterPhase)
{
    // std::cout << "[doPhaseFollows()]" << std::endl;

    // std::cout << "      beforePhase: " << beforePhase << std::endl;
    // std::cout << "      afterPhase: " << afterPhase << std::endl;

    bool followingPhases = false;
    for (int i = 0; i < numSwingPhases; i++)
    {
        if (modeSequenceString_[2 * i] == beforePhase && 
            modeSequenceString_[2 * ((i + 1) % numSwingPhases)] == afterPhase)
            followingPhases = true;
    }

    // std::cout << "      followingPhase: " << followingPhases << std::endl;

    return followingPhases;
}

bool ContactSequence::isLegInSwing(const short & legIdx)
{
    return isLegInSwing(legIdx, swingPhaseCounter_);
}

bool ContactSequence::isLegInSwing(const short & legIdx, const short & swingPhaseCounter)
{
    std::string currentMode = modeSequenceString_[2 * swingPhaseCounter];

    return (currentMode.find(LegModeScheduleShort.at(legIdx)) == std::string::npos);
}


}  // namespace mmp

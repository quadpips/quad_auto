#pragma once

#include <vector>
#include <ocs2_core/misc/LoadData.h>
#include <mmp_quadruped/planning/utils.h>


namespace mmp {

/****************************ContactSequence****************************/
class ContactSequence
{
    public:
        /**
         * @brief Construct a new Contact Sequence object
         *
         * @param gaitCommandFile file containing gait commands
         * @param gait gait name
         */
        ContactSequence(const std::string & gaitCommandFile, const std::string & gait);

        /**
         * @brief Check if the phases follow each other in the mode sequence
         *
         * @param beforePhase phase before
         * @param afterPhase phase after
         * @return true if the phases follow each other
         */
        bool doPhasesFollow(const std::string & beforePhase, const std::string & afterPhase);

        /**
         * @brief Check if the leg is in swing phase
         *
         * @param legIdx leg index
         * @return true if the leg is in swing phase
         */
        bool isLegInSwing(const short & legIdx);

        /**
         * @brief Check if the leg is in swing phase
         *
         * @param legIdx leg index
         * @param swingPhaseCounter swing phase counter
         * @return true if the leg is in swing phase
         */
        bool isLegInSwing(const short & legIdx, const short & swingPhaseCounter);

        /**
         * @brief Get the next switching time in the mode sequence
         *
         * @return next switching time
         */
        std::string getNextSwitchingTime() { return switchingTimesString_[(2 * swingPhaseCounter_ + 1) % switchingTimesString_.size()]; }
        
        /**
         * @brief Get the next next switching time in the mode sequence
         *
         * @return next next switching time
         */
        std::string getNextNextSwitchingTime() { return switchingTimesString_[(2 * swingPhaseCounter_ + 2) % switchingTimesString_.size()]; }
        
        /**
         * @brief Get the current mode in the mode sequence
         *
         * @return current mode
         */
        std::string getCurrentMode() { return modeSequenceString_[2 * swingPhaseCounter_]; }
        
        /**
         * @brief Get the next mode in the mode sequence
         *
         * @return next mode
         */
        std::string getNextMode() { return modeSequenceString_[(2 * swingPhaseCounter_ + 1) % modeSequenceString_.size()]; }

        
        std::string getNextMode(const std::string & currentMode) 
        { 
            for (int i = 0; i < modeSequenceString_.size(); i++)
            {
                if (modeSequenceString_.at(i) == currentMode)
                {
                    return modeSequenceString_.at( (i + 2) % modeSequenceString_.size() ); // Skip over stance
                }
            }
            throw std::runtime_error("[getNextMode] currentMode not found in modeSequenceString_");
        }
        
        /**
         * @brief Get the number of swing phases in the mode sequence
         *
         * @return number of swing phases in the mode sequence
         */
        short getNumSwingPhases() { return numSwingPhases; } 

        /**
         * @brief Get the current swing phase counter in the mode sequence
         *
         * @return current swing phase counter in the mode sequence
         */
        short getSwingPhaseCounter() { return swingPhaseCounter_;}

        /**
         * @brief Advance the swing phase counter in the mode sequence
         */
        void advanceCounter() { swingPhaseCounter_ = (swingPhaseCounter_ + 1) % numSwingPhases; } 

        /**
         * @brief Reset the swing phase counter in the mode sequence
         */
        void reset() { swingPhaseCounter_ = 0; }

        /**
         * @brief Get the swing time of the gait
         *
         * @return swing time of the gait
         */
        double getSwingTime() { return swingTime; }

        /**
         * @brief Get the stance time of the gait
         *
         * @return stance time of the gait
         */
        double getStanceTime() { return stanceTime; }

    private:

        std::vector<std::string> modeSequenceString_; /**< mode sequence string */
        std::vector<std::string> switchingTimesString_; /**< switching times string */

        int swingPhaseCounter_ = 0; /**< swing phase counter */
        int numSwingPhases = 0; /**< number of swing phases */

        double swingTime = -1.0; /** swing time of gait, assuming constant across gait right now */
        double stanceTime = -1.0; /** stance time of gait, assuming constant across gait right now */        
};

}  // namespace mmp

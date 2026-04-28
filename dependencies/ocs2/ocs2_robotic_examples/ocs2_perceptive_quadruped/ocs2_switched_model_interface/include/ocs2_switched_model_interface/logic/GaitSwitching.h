//
// Created by rgrandia on 15.03.20.
//

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/Gait.h"

namespace switched_model {

/**
 * Selects the next gait from iterators. Repeating the final gait when the final gait is reached.
 * @tparam GaitIt : Gait iterator
 * @param currentGait : iterator pointing to the current gait
 * @param pastTheEndGait : iterator pointing part the end of the last gait to be considered. (e.g. obtained from vector::end())
 * @return next gait in the sequence
 */
template <typename GaitIt>
GaitIt nextGait(GaitIt currentGait, GaitIt pastTheEndGait) {
  GaitIt nextGait = currentGait + 1;
  if (nextGait == pastTheEndGait) {
    return currentGait;
  } else {
    return nextGait;
  }
}

/**
 * Recursively progresses to the next gait in a sequence while keeping track of the phase variable.
 * When the final gait is reached, it is repeated as long as required by the specified dt.
 *
 * @tparam GaitIt : Gait iterator
 * @param phase : phase variable in the current gait.
 * @param dt : time to progress in the gait sequence
 * @param currentGait : iterator pointing to the current gait
 * @param pastTheEndGait : iterator pointing part the end of the last gait to be considered. (e.g. obtained from vector::end())
 * @return {phase in new gait, iterator to newly active gait.
 */
template <typename GaitIt>
std::pair<scalar_t, GaitIt> advancePhase(scalar_t phase, scalar_t dt, GaitIt currentGait, GaitIt pastTheEndGait) {
  assert(isValidPhase(phase));
  assert(dt >= 0);

  // Phase change within current gait
  scalar_t dphase = dt / currentGait->duration;

  if (phase + dphase < 1.0) {  // Advance within current gait
    phase += dphase;
    return {phase, currentGait};
  } else {  // Advance to next gait
    const scalar_t dtRemaining = std::max(dt - timeLeftInGait(phase, *currentGait), 0.0);
    const GaitIt nexGait = nextGait(currentGait, pastTheEndGait);
    // Recurse by setting the phase to the beginning of the next phase
    return advancePhase(0.0, dtRemaining, nexGait, pastTheEndGait);
  }
}

template <typename GaitIt>
ocs2::ModeSchedule getModeSchedule(scalar_t phase, scalar_t t0, scalar_t timeHorizon, GaitIt currentGait, GaitIt pastTheEndGait) 
{
  // std::cout << "[GaitSwitching::getModeSchedule]" << std::endl;
  assert(isValidPhase(phase));

  // std::cout << "[GaitSwitching::getModeSchedule]  phase: " << phase << std::endl;
  // std::cout << "[GaitSwitching::getModeSchedule]  t0: " << t0 << std::endl;
  // std::cout << "[GaitSwitching::getModeSchedule]  timeHorizon: " << timeHorizon << std::endl;

  // std::cout << "[GaitSwitching::getModeSchedule]  currentGait: " << std::endl;
  // std::cout << "   duration: " << currentGait->duration << std::endl;
  // std::cout << "   modeSequence: " << std::endl;
  // for (const auto & mode : currentGait->modeSequence)
  // {
  //     std::cout << "     " << mode << std::endl;
  // }
  // std::cout << "   eventPhases: " << std::endl;
  // for (const auto & phase : currentGait->eventPhases)
  // {
  //     std::cout << "     " << phase << std::endl;
  // }

  // std::cout << "[GaitSwitching::getModeSchedule]  pastTheEndGait " << std::endl;
  // std::cout << "   duration: " << pastTheEndGait->duration << std::endl;
  // std::cout << "   modeSequence: " << std::endl;
  // for (const auto & mode : pastTheEndGait->modeSequence)
  // {
  //     std::cout << "     " << mode << std::endl;
  // }
  // std::cout << "   eventPhases: " << std::endl;
  // for (const auto & phase : pastTheEndGait->eventPhases)
  // {
  //     std::cout << "     " << phase << std::endl;
  // }

  // Initialize with the current mode
  std::vector<scalar_t> evenTimes = {};
  std::vector<size_t> modeSchedule = {getModeFromPhase(phase, *currentGait)};

  const scalar_t tend = t0 + timeHorizon;

  // std::cout << "[GaitSwitching::getModeSchedule]  tend: " << tend << std::endl;

  scalar_t t = t0;
  while (t < tend) 
  {
    // std::cout << "[GaitSwitching::getModeSchedule]  starting loop" << std::endl;
    scalar_t dt = timeLeftInMode(phase, *currentGait);
    // std::cout << "[GaitSwitching::getModeSchedule]    dt: " << dt << std::endl;
    t += dt;
    // std::cout << "[GaitSwitching::getModeSchedule]    t: " << t << std::endl;

    if (t < tend) // Next event is within horizon: Add the event time and advance the phase to that event
    {
      // std::cout << "[GaitSwitching::getModeSchedule]    advancing phase" << std::endl;
      std::tie(phase, currentGait) = advancePhase(phase, dt, currentGait, pastTheEndGait);
      // std::cout << "[GaitSwitching::getModeSchedule]    new phase: " << phase << std::endl;
      // std::cout << "[GaitSwitching::getModeSchedule]    new currentGait duration: " << currentGait->duration << std::endl;
      // std::cout << "[GaitSwitching::getModeSchedule]    new currentGait modeSequence: " << std::endl;
      // for (const auto & mode : currentGait->modeSequence)
      // {
      //     std::cout << "     " << mode << std::endl;
      // }
      // std::cout << "[GaitSwitching::getModeSchedule]    new currentGait eventPhases: " << std::endl;
      // for (const auto & phase : currentGait->eventPhases)
      // {
      //     std::cout << "     " << phase << std::endl;
      // }

      size_t mode = getModeFromPhase(phase, *currentGait);

      // std::cout << "[GaitSwitching::getModeSchedule]    new mode: " << mode << std::endl;
      
      if (mode != modeSchedule.back()) // Only add the mode if it is a switch w.r.t the last mode.
      {
        // std::cout << "[GaitSwitching::getModeSchedule]    adding new mode and event time" << std::endl;
        // std::cout << "      event time: " << t << std::endl;
        // std::cout << "      mode: " << mode << std::endl;
        evenTimes.push_back(t);
        modeSchedule.push_back(mode);
      }
    }
  }

  // std::cout << "  mode schedule: " << std::endl;
  // for (const auto & mode : modeSchedule)
  // {
  //     std::cout << "   " << mode << std::endl;
  // }
  
  // std::cout << "  event times: " << std::endl;
  // for (const auto & time : evenTimes)
  // {
  //     std::cout << "   " << time << std::endl;
  // }

  return {evenTimes, modeSchedule};
}

}  // namespace switched_model

//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"
#include "ocs2_switched_model_interface/foot_planner/KinematicFootPlacementPenalty.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings,
                                               const KinematicsModelBase<scalar_t>& kinematicsModel,
                                               const InverseKinematicsModelBase* inverseKinematicsModelPtr)
    : settings_(std::move(settings)),
      kinematicsModel_(kinematicsModel.clone()),
      inverseKinematicsModelPtr_(nullptr),
      terrainModel_(nullptr) {
  if (inverseKinematicsModelPtr != nullptr) {
    inverseKinematicsModelPtr_.reset(inverseKinematicsModelPtr->clone());
  }
}

void SwingTrajectoryPlanner::updateTerrain(std::unique_ptr<TerrainModel> terrainModel) 
{
  terrainModel_ = std::move(terrainModel);
}

const SignedDistanceField* SwingTrajectoryPlanner::getSignedDistanceField() const 
{
  if (terrainModel_) {
    return terrainModel_->getSignedDistanceField();
  } else {
    return nullptr;
  }
}

void SwingTrajectoryPlanner::updateSwingMotions(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                                                const ocs2::TargetTrajectories& targetTrajectories,
                                                const ocs2::ModeSchedule& modeSchedule) 
{
  // std::cout << "[SwingTrajectoryPlanner::updateSwingMotions]" << std::endl;

  if (!terrainModel_) {
    throw std::runtime_error("[SwingTrajectoryPlanner] terrain cannot be null. Update the terrain before planning swing motions");
  }

  // std::cout << "    initTime: " << initTime << std::endl;

  // std::cout << "    finalTime: " << finalTime << std::endl;

  // Need a copy to
  // 1. possibly overwrite joint references later (adapted with inverse kinematics)
  // 2. ensure a maximum interval between references points.
  // 3. unsure we have samples at start and end of the MPC horizon.
  subsampleReferenceTrajectory(targetTrajectories, initTime, finalTime);

  const feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg = extractContactTimingsPerLeg(modeSchedule);

  const auto basePose = getBasePose(currentState);
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  // std::cout << "    leg loop: " << std::endl;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) 
  {
    // std::string legName;
    // if (leg == 0)
    //   legName = "LF";
    // else if (leg == 1)
    //   legName = "RF";
    // else if (leg == 2)
    //   legName = "LH";
    // else if (leg == 3)
    //   legName = "RH";
    // else
    //   throw std::runtime_error("[SwingTrajectoryPlanner] Invalid leg index: " + std::to_string(leg));

    // std::cout << "  leg: " << leg << " (name: " << legName << ")" << std::endl;

    const auto& contactTimings = contactTimingsPerLeg[leg];

    // Update last contacts
    if (!contactTimings.empty()) 
    {
      // std::cout << "        !contactTimings.empty()" << std::endl;

      if (startsWithStancePhase(contactTimings)) 
      {
        // std::cout << "          startsWithStancePhase(contactTimings)" << std::endl;
        // If currently in contact -> update expected liftoff.
        if (hasEndTime(contactTimings.front())) 
        {
          // std::cout << "            hasEndTime(contactTimings.front())" << std::endl;
          updateLastContact(leg, contactTimings.front().end, feetPositions[leg], *terrainModel_);
        } else 
        {  // Expected liftoff unknown, set to end of horizon
          // std::cout << "            !hasEndTime(contactTimings.front())" << std::endl;
          updateLastContact(leg, finalTime, feetPositions[leg], *terrainModel_);
        }
      } else 
      { // If currently in swing -> verify that liftoff was before the horizon. If not, assume liftoff happened exactly at initTime
        // std::cout << "          !startsWithStancePhase(contactTimings)" << std::endl;
        if (lastContacts_[leg].first > initTime) 
        {
          // std::cout << "            lastContacts_[leg].first > initTime" << std::endl;
          updateLastContact(leg, initTime, feetPositions[leg], *terrainModel_);
        }
      }
    }

    // std::cout << "  contactTimings: " << std::endl;
    // for (const auto& contactTiming : contactTimings) 
    // {
    //   std::cout << "    start: " << contactTiming.start << ", end: " << contactTiming.end << std::endl;
    // }

    // Select heuristic footholds.
    // std::cout << "  pre-selectHeuristicFootholds" << std::endl;
    heuristicFootholdsPerLeg_[leg] = selectHeuristicFootholds(leg, contactTimings, targetTrajectories, initTime, currentState, finalTime);
    // std::cout << "  post-selectHeuristicFootholds" << std::endl;

    // std::cout << "  heuristicFootholds: " << std::endl;
    // for (const auto& foothold : heuristicFootholdsPerLeg_[leg]) 
    // {
      // std::cout << "    " << foothold.transpose() << std::endl;
    // }

    // Select terrain constraints based on the heuristic footholds.
    // std::cout << "  pre-selectNominalFootholdTerrain" << std::endl;
    nominalFootholdsPerLeg_[leg] = selectNominalFootholdTerrain(leg, contactTimings, heuristicFootholdsPerLeg_[leg], targetTrajectories,
                                                                initTime, currentState, finalTime, *terrainModel_);
    // std::cout << "  post-selectNominalFootholdTerrain" << std::endl;

    // Create swing trajectories
    if (settings_.swingTrajectoryFromReference) 
    {
      // std::cout << "  swingTrajectoryFromReference" << std::endl;
      std::tie(feetNormalTrajectoriesEvents_[leg], feetNormalTrajectories_[leg]) =
          extractSwingTrajectoriesFromReference(leg, contactTimings, finalTime);
    } else 
    {
      // std::cout << "  !swingTrajectoryFromReference" << std::endl;    
      std::tie(feetNormalTrajectoriesEvents_[leg], feetNormalTrajectories_[leg]) =
          generateSwingTrajectories(leg, contactTimings, finalTime);
    }
  }

  if (inverseKinematicsModelPtr_ && !settings_.swingTrajectoryFromReference) 
  {
    // std::cout << "  adaptJointReferencesWithInverseKinematics" << std::endl;
    adaptJointReferencesWithInverseKinematics(finalTime);
  }

  // std::cout << "  done with updateSwingMotions" << std::endl;

  return;
}

const FootPhase& SwingTrajectoryPlanner::getFootPhase(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetNormalTrajectoriesEvents_[leg], time);
  return *feetNormalTrajectories_[leg][index];
}

auto SwingTrajectoryPlanner::generateSwingTrajectories(int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const
    -> std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> 
{
  // std::cout << "      [SwingTrajectoryPlanner::generateSwingTrajectories]" << std::endl;
  // std::cout << "          leg: " << leg << std::endl;

  std::vector<scalar_t> eventTimes;
  std::vector<std::unique_ptr<FootPhase>> footPhases;

  // First swing phase
  if (startsWithSwingPhase(contactTimings)) 
  {
    SwingPhase::SwingEvent liftOff{lastContacts_[leg].first, settings_.liftOffVelocity, &lastContacts_[leg].second};
    SwingPhase::SwingEvent touchDown = [&] 
    {
      if (touchesDownAtLeastOnce(contactTimings)) 
      {
        return SwingPhase::SwingEvent{contactTimings.front().start, settings_.touchDownVelocity,
                                      &nominalFootholdsPerLeg_[leg].front().plane};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
      }
    }();

    SwingPhase::SwingProfile swingProfile = getDefaultSwingProfile();
    applySwingMotionScaling(liftOff, touchDown, swingProfile);

    SwingPhase * swingPhase = new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get());
    // std::cout << "          adding swing phase (1):" << std::endl;
    // std::cout << "              liftOff time:   " << liftOff.time << std::endl;
    // std::cout << "              liftOff pos:    " << swingPhase->getPositionInWorld(liftOff.time).transpose() << std::endl;
    // std::cout << "              liftOff vel:    " << swingPhase->getVelocityInWorld(liftOff.time).transpose() << std::endl;
    // std::cout << "              liftOff norm:   " << swingPhase->normalDirectionInWorldFrame(liftOff.time).transpose() << std::endl;
    // std::cout << "              apex time:      " << (liftOff.time + touchDown.time) / 2.0 << std::endl;
    // std::cout << "              apex pos:       " << swingPhase->getPositionInWorld((liftOff.time + touchDown.time) / 2.0).transpose() << std::endl;
    // std::cout << "              apex vel:       " << swingPhase->getVelocityInWorld((liftOff.time + touchDown.time) / 2.0).transpose() << std::endl;
    // std::cout << "              apex norm:      " << swingPhase->normalDirectionInWorldFrame((liftOff.time + touchDown.time) / 2.0).transpose() << std::endl;
    // std::cout << "              touchDown time: " << touchDown.time << std::endl;
    // std::cout << "              touchDown pos:  " << swingPhase->getPositionInWorld(touchDown.time).transpose() << std::endl;
    // std::cout << "              touchDown vel:  " << swingPhase->getVelocityInWorld(touchDown.time).transpose() << std::endl;
    // std::cout << "              touchDown norm: " << swingPhase->normalDirectionInWorldFrame(touchDown.time).transpose() << std::endl;
    footPhases.emplace_back(swingPhase);
  }

  // Loop through contact phases
  // std::cout << "      contact timings loop" << std::endl;
  for (int i = 0; i < contactTimings.size(); ++i) 
  {
    const auto& currentContactTiming = contactTimings[i];
    const ConvexTerrain& nominalFoothold = nominalFootholdsPerLeg_[leg][i];

    // std::cout << "        currentContactTiming.start: " << currentContactTiming.start << std::endl;
    // std::cout << "        nominalFoothold.plane.positionInWorld: " << nominalFoothold.plane.positionInWorld.transpose() << std::endl;

    // If phase starts after the horizon, we don't need to plan for it
    if (currentContactTiming.start > finalTime) {
      break;
    }

    // generate contact phase
    if (hasStartTime(currentContactTiming)) {
      eventTimes.push_back(currentContactTiming.start);
    }

    StancePhase * stancePhase = new StancePhase(nominalFoothold, settings_.terrainMargin);
    // std::cout << "          adding stance phase:" << std::endl;
    // std::cout << "              start time: " << currentContactTiming.start << std::endl;
    // std::cout << "              end time: " << currentContactTiming.end << std::endl;
    // std::cout << "              nominal foothold: " << stancePhase->nominalFootholdLocation().transpose() << std::endl;
    footPhases.emplace_back(stancePhase);

    // If contact phase extends beyond the horizon, we can stop planning.
    if (!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) {
      break;
    }

    // generate swing phase afterwards
    SwingPhase::SwingEvent liftOff{currentContactTiming.end, settings_.liftOffVelocity, &nominalFoothold.plane};
    SwingPhase::SwingEvent touchDown = [&] {
      const bool nextContactExists = (i + 1) < contactTimings.size();
      if (nextContactExists) {
        return SwingPhase::SwingEvent{contactTimings[i + 1].start, settings_.touchDownVelocity, &nominalFootholdsPerLeg_[leg][i + 1].plane};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
      }
    }();

    SwingPhase::SwingProfile swingProfile = getDefaultSwingProfile();
    applySwingMotionScaling(liftOff, touchDown, swingProfile);

    eventTimes.push_back(currentContactTiming.end);

    SwingPhase * swingPhase = new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get());
    // std::cout << "          adding swing phase (2):" << std::endl;
    // std::cout << "              liftOff time:   " << liftOff.time << std::endl;
    // std::cout << "              liftOff pos:    " << swingPhase->getPositionInWorld(liftOff.time).transpose() << std::endl;
    // std::cout << "              liftOff vel:    " << swingPhase->getVelocityInWorld(liftOff.time).transpose() << std::endl;
    // std::cout << "              liftOff norm:   " << swingPhase->normalDirectionInWorldFrame(liftOff.time).transpose() << std::endl;
    // std::cout << "              apex time:      " << (liftOff.time + touchDown.time) / 2.0 << std::endl;
    // std::cout << "              apex pos:       " << swingPhase->getPositionInWorld((liftOff.time + touchDown.time) / 2.0).transpose() << std::endl;
    // std::cout << "              apex vel:       " << swingPhase->getVelocityInWorld((liftOff.time + touchDown.time) / 2.0).transpose() << std::endl;
    // std::cout << "              apex norm:      " << swingPhase->normalDirectionInWorldFrame((liftOff.time + touchDown.time) / 2.0).transpose() << std::endl;
    // std::cout << "              touchDown time: " << touchDown.time << std::endl;
    // std::cout << "              touchDown pos:  " << swingPhase->getPositionInWorld(touchDown.time).transpose() << std::endl;
    // std::cout << "              touchDown vel:  " << swingPhase->getVelocityInWorld(touchDown.time).transpose() << std::endl;
    // std::cout << "              touchDown norm: " << swingPhase->normalDirectionInWorldFrame(touchDown.time).transpose() << std::endl;
    footPhases.emplace_back(swingPhase);
  }

  // std::cout << "        eventTimes: " << std::endl;
  // for (int i = 0; i < eventTimes.size(); ++i) 
  // {
  //   std::cout << "          " << i << ": " << eventTimes[i] << std::endl;
  // }

  // std::cout << "        footPhases: " << std::endl;
  // for (int i = 0; i < footPhases.size(); ++i) 
  // {
  //   if (footPhases[i]->contactFlag())   // StancePhase
  //   {     
  //     std::cout << "          " << i << ": Stance" << std::endl;
  //     std::cout << "                          nominal foothold: " << footPhases[i]->nominalFootholdLocation().transpose() << std::endl;
  //   } else // SwingPhase 
  //   {  
  //     std::cout << "          " << i << ": Swing" << std::endl;
  //     for (scalar_t t = 0.0; t < finalTime; t += 0.01)
  //     {
  //       std::cout << "                        t: " << t << ", position: " << footPhases[i]->getPositionInWorld(t).transpose() << std::endl;
  //     }
  //   }
  // }


  // if (footPhases.size() == 1)
  // {
  //   for (scalar_t t = 0; t < 0.50; t += 0.01)
  //     std::cout << "            t: " << t << ", position: " << footPhases[0]->getPositionInWorld(t).transpose()
  //                                        << ", normal: " << footPhases[0]->normalDirectionInWorldFrame(t).transpose() << std::endl; 
  // }

  // if (footPhases.size() == 3)
  // {
  //   for (int i = 0; i < footPhases.size(); i++)
  //   {
  //     if (footPhases[i]->contactFlag())       // StancePhase
  //       std::cout << "          " << i << ": Stance" << std::endl;
  //     else // SwingPhase
  //       std::cout << "          " << i << ": Swing" << std::endl;

  //     if (i == 0)
  //     {
  //       for (scalar_t t = 0; t < 0.05; t += 0.01)
  //         std::cout << "            t: " << t << ", position: " << footPhases[i]->getPositionInWorld(t).transpose() 
  //                                        << ", normal: " << footPhases[i]->normalDirectionInWorldFrame(t).transpose() << std::endl; 
  //     } else if (i == 1)
  //     {
  //       for (scalar_t t = 0.05; t < 0.45; t += 0.01)
  //         std::cout << "            t: " << t << ", position: " << footPhases[i]->getPositionInWorld(t).transpose()
  //                                        << ", normal: " << footPhases[i]->normalDirectionInWorldFrame(t).transpose() << std::endl; 
  //     } else if (i == 2)
  //     {
  //       for (scalar_t t = 0.45; t < 0.50; t += 0.01)
  //         std::cout << "            t: " << t << ", position: " << footPhases[i]->getPositionInWorld(t).transpose()
  //                                        << ", normal: " << footPhases[i]->normalDirectionInWorldFrame(t).transpose() << std::endl; 
  //     }
  //   }
  // }

  return std::make_pair(eventTimes, std::move(footPhases));
}

std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> SwingTrajectoryPlanner::extractSwingTrajectoriesFromReference(
    int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const 
{
  // std::cout << "[extractSwingTrajectoriesFromReference]" << std::endl;
  // std::cout << "    leg: " << leg << std::endl;

  std::vector<scalar_t> eventTimes;
  std::vector<std::unique_ptr<FootPhase>> footPhases;

  // First swing phase
  if (startsWithSwingPhase(contactTimings)) 
  {
    // std::cout << "      starting with swing phase" << std::endl;
    scalar_t liftOffTime{lastContacts_[leg].first};
    scalar_t touchDownTime = [&] {
      if (touchesDownAtLeastOnce(contactTimings)) {
        return contactTimings.front().start;
      } else {
        return finalTime + settings_.referenceExtensionAfterHorizon;
      }
    }();

    footPhases.push_back(extractExternalSwingPhase(leg, liftOffTime, touchDownTime));
  } else
  {
    // std::cout << "      starting with stance phase" << std::endl;
  }

  // Loop through contact phases
  for (int i = 0; i < contactTimings.size(); ++i) 
  {
    // std::cout << "        contact phase " << i << ": " << std::endl;
    // std::cout << "          start: " << contactTimings[i].start << ", end: " << contactTimings[i].end << std::endl;

    const auto& currentContactTiming = contactTimings[i];
    const ConvexTerrain& nominalFoothold = nominalFootholdsPerLeg_[leg][i];

    // If phase starts after the horizon, we don't need to plan for it
    if (currentContactTiming.start > finalTime) 
    {
      // std::cout << "          phase starts after the horizon, breaking" << std::endl;
      break;
    } else
    {
      // std::cout << "          phase starts before the horizon, continuing" << std::endl;
    }

    // generate contact phase
    if (hasStartTime(currentContactTiming)) 
    {
      // std::cout << "          hasStartTime(currentContactTiming)" << std::endl;
      eventTimes.push_back(currentContactTiming.start);
    } else
    {
      // std::cout << "          !hasStartTime(currentContactTiming)" << std::endl;
    }

    footPhases.emplace_back(new StancePhase(nominalFoothold, settings_.terrainMargin));

    // If contact phase extends beyond the horizon, we can stop planning.
    if (!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) 
    {
      // std::cout << "          contact phase extends beyond the horizon" << std::endl;
      break;
    } else
    {
      // std::cout << "          contact phase ends before the horizon" << std::endl;
    }

    // std::cout << "          generating swing phase afterwards" << std::endl;
    // generate swing phase afterwards
    scalar_t liftOffTime{currentContactTiming.end};
    scalar_t touchDownTime = [&] {
      const bool nextContactExists = (i + 1) < contactTimings.size();
      if (nextContactExists) {
        return contactTimings[i + 1].start;
      } else {
        return finalTime + settings_.referenceExtensionAfterHorizon;
      }
    }();

    // std::cout << "          liftOffTime: " << liftOffTime << ", touchDownTime: " << touchDownTime << std::endl;
    eventTimes.push_back(currentContactTiming.end);
    footPhases.push_back(extractExternalSwingPhase(leg, liftOffTime, touchDownTime));
  }

  // std::cout << "        eventTimes: " << std::endl;
  // for (int i = 0; i < eventTimes.size(); ++i) {
    // std::cout << "          " << i << ": " << eventTimes[i] << std::endl;
  // }

  // std::cout << "        footPhases: " << std::endl;
  // for (int i = 0; i < footPhases.size(); ++i)
  // {
  //   if (footPhases[i]->contactFlag()) {  // StancePhase
  //     std::cout << "          " << i << ": Stance" << std::endl;
  //     std::cout << "                          nominal foothold: " << footPhases[i]->nominalFootholdLocation().transpose() << std::endl;
  //   } else {  // SwingPhase
  //     std::cout << "          " << i << ": Swing" << std::endl;
  //   }
  // }

  return std::make_pair(eventTimes, std::move(footPhases));
}

void SwingTrajectoryPlanner::applySwingMotionScaling(SwingPhase::SwingEvent& liftOff, SwingPhase::SwingEvent& touchDown,
                                                     SwingPhase::SwingProfile& swingProfile) const 
{
  // std::cout << "[SwingTrajectoryPlanner::applySwingMotionScaling]" << std::endl;

  const scalar_t scaling = [&]() 
  {
    if (std::isnan(liftOff.time) || std::isnan(touchDown.time)) 
    {
      return 1.0;
    } else 
    {
      return std::min(1.0, (touchDown.time - liftOff.time) / settings_.swingTimeScale);
    }
  }();

  // std::cout << "scaling: " << scaling << std::endl;

  if (scaling < 1.0) 
  {
    liftOff.velocity *= scaling;
    touchDown.velocity *= scaling;
    swingProfile.sdfMidswingMargin = scaling * settings_.sdfMidswingMargin;
    for (auto& node : swingProfile.nodes) 
    {
      node.swingHeight *= scaling;
      node.normalVelocity *= scaling;
    }
  }

  // for (auto& node : swingProfile.nodes) 
  // {
  //   std::cout << "swing height: " << node.swingHeight << std::endl;
  // }
}

std::unique_ptr<ExternalSwingPhase> SwingTrajectoryPlanner::extractExternalSwingPhase(int leg, scalar_t liftOffTime,
                                                                                      scalar_t touchDownTime) const 
{
  // std::cerr << "[SwingTrajectoryPlanner::extractExternalSwingPhase] leg: " << leg
  //           << ", liftOffTime: " << liftOffTime << ", touchDownTime: " << touchDownTime << std::endl;

  std::vector<scalar_t> time;
  std::vector<vector3_t> positions;
  std::vector<vector3_t> velocities;

  // std::cerr << "    targetTrajectories_ state trajectory size: " << targetTrajectories_.stateTrajectory.size() << std::endl;
  // std::cerr << "    targetTrajectories_ input trajectory size: " << targetTrajectories_.inputTrajectory.size() << std::endl;
  // std::cerr << "    targetTrajectories_ time trajectory size: " << targetTrajectories_.timeTrajectory.size() << std::endl;

  const auto liftoffIndex = ocs2::LinearInterpolation::timeSegment(liftOffTime, targetTrajectories_.timeTrajectory);
  const auto touchdownIndex = ocs2::LinearInterpolation::timeSegment(touchDownTime, targetTrajectories_.timeTrajectory);

  // std::cerr << "    liftoffIndex: " << liftoffIndex.first << ", touchdownIndex: " << touchdownIndex.first << std::endl;

  // std::cerr << "    extracting lift off ..." << std::endl;

  // liftoff
  if (liftOffTime < targetTrajectories_.timeTrajectory[liftoffIndex.first + 1]) 
  {
    // std::cerr << "  lift off time before reference trajectory" << std::endl;
    const vector_t state = ocs2::LinearInterpolation::interpolate(liftoffIndex, targetTrajectories_.stateTrajectory);
    const vector_t input = ocs2::LinearInterpolation::interpolate(liftoffIndex, targetTrajectories_.inputTrajectory);
    time.push_back(liftOffTime);
    positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
    velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
                                                                     getJointPositions(state), getJointVelocities(input)));
  
  }

  // std::cerr << "    extracting intermediate points ..." << std::endl;

  // intermediate
  for (int k = liftoffIndex.first + 1; k < touchdownIndex.first; ++k) {
    const auto& state = targetTrajectories_.stateTrajectory[k];
    time.push_back(targetTrajectories_.timeTrajectory[k]);
    positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
    velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
                                                                     getJointPositions(state),
                                                                     getJointVelocities(targetTrajectories_.inputTrajectory[k])));
  }

  // std::cerr << "    extracting touchdown ..." << std::endl;

  // empty corner case

  if (time.empty())
  {
    // std::cerr << "    targetTrajectories_: " << std::endl;
    // for (int i = 0; i < targetTrajectories_.timeTrajectory.size(); i++)
    // {
    //   std::cerr << "      time: " << targetTrajectories_.timeTrajectory[i] << std::endl;
    //   std::cerr << "        state: " << targetTrajectories_.stateTrajectory[i].transpose() << std::endl;
    //   std::cerr << "        input: " << targetTrajectories_.inputTrajectory[i].transpose() << std::endl;
    // }
    throw std::runtime_error("[SwingTrajectoryPlanner::extractExternalSwingPhase] Extracted swing phase is empty!");
    // std::cerr << "    Extracted swing phase is empty" << std::endl;
  }

  // if (time.empty()) 
  // {
  //   std::cerr << "    time is empty!" << std::endl;
  //   const vector_t state = ocs2::LinearInterpolation::interpolate(liftoffIndex, targetTrajectories_.stateTrajectory);
  //   const vector_t input = ocs2::LinearInterpolation::interpolate(liftoffIndex, targetTrajectories_.inputTrajectory);
  //   time.push_back(liftOffTime);
  //   positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
  //   velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
  //                                                                    getJointPositions(state), getJointVelocities(input)));
  // }

  // std::cerr << "    time.back(): " << time.back() << std::endl; 
  // std::cerr << "    touchDownTime: " << touchDownTime << std::endl;

  // touchdown
  if (time.back() < touchDownTime) 
  {
    const vector_t state = ocs2::LinearInterpolation::interpolate(touchdownIndex, targetTrajectories_.stateTrajectory);
    // std::cerr << "    state: " << state.transpose() << std::endl;
    const vector_t input = ocs2::LinearInterpolation::interpolate(touchdownIndex, targetTrajectories_.inputTrajectory);
    // std::cerr << "    input: " << input.transpose() << std::endl;
    time.push_back(touchDownTime);

    // std::cerr << "    extracting foot position at touchdown ..." << std::endl;

    positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
    
    // std::cerr << "    extracting foot velocity at touchdown ..." <<  std::endl;
    
    velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
                                                                     getJointPositions(state), getJointVelocities(input)));
  }

  // std::cerr << "    extraction complete." << std::endl;

  return std::unique_ptr<ExternalSwingPhase>(new ExternalSwingPhase(move(time), move(positions), move(velocities)));
}

std::vector<vector3_t> SwingTrajectoryPlanner::selectHeuristicFootholds(int leg, const std::vector<ContactTiming>& contactTimings,
                                                                        const ocs2::TargetTrajectories& targetTrajectories,
                                                                        scalar_t initTime, const comkino_state_t& currentState,
                                                                        scalar_t finalTime) const 
{
  // Zmp preparation : measured state
  // const auto initBasePose = getBasePose(currentState);
  // const auto initBaseOrientation = getOrientation(initBasePose);
  // const auto initBaseTwistInBase = getBaseLocalVelocities(currentState);
  // const auto initBaseLinearVelocityInWorld = rotateVectorBaseToOrigin(getLinearVelocity(initBaseTwistInBase), initBaseOrientation);

  // Zmp preparation : desired state
  // const vector_t initDesiredState = targetTrajectories.getDesiredState(initTime);
  // const base_coordinate_t initDesiredBasePose = initDesiredState.head<BASE_COORDINATE_SIZE>();
  // const auto initDesiredOrientation = getOrientation(initDesiredBasePose);
  // const base_coordinate_t initDesiredBaseTwistInBase = initDesiredState.segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
  // const auto initDesiredBaseLinearVelocityInWorld =
  //     rotateVectorBaseToOrigin(getLinearVelocity(initDesiredBaseTwistInBase), initDesiredOrientation);

  // Compute zmp / inverted pendulum foot placement offset: delta p = sqrt(h / g) * (v - v_des)
  // scalar_t pendulumFrequency = std::sqrt(settings_.invertedPendulumHeight / 9.81);
  // scalar_t zmpX = pendulumFrequency * (initBaseLinearVelocityInWorld.x() - initDesiredBaseLinearVelocityInWorld.x());
  // scalar_t zmpY = pendulumFrequency * (initBaseLinearVelocityInWorld.y() - initDesiredBaseLinearVelocityInWorld.y());
  // const vector3_t zmpReactiveOffset = {zmpX, zmpY, 0.0};

  // Heuristic footholds to fill
  std::vector<vector3_t> heuristicFootholds;

  // Heuristic foothold is equal to current foothold for legs in contact
  if (startsWithStancePhase(contactTimings)) 
  {
    heuristicFootholds.push_back(lastContacts_[leg].second.positionInWorld);
    // std::cout << "  starts with stance phase, adding last foot position: " << lastContacts_[leg].second.positionInWorld.transpose() << std::endl;
  }

  // For future contact phases, use TargetTrajectories at halve the contact phase
  int contactCount = 0;
  for (const auto& contactPhase : contactTimings) 
  {
    if (hasStartTime(contactPhase)) 
    {
      // std::cout << "  contactPhase.start: " << contactPhase.start << std::endl;
      const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
      // std::cout << "  contactEndTime: " << contactEndTime << std::endl;
      const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

      // Compute foot position from cost desired trajectory
      const vector_t state = targetTrajectories.getDesiredState(middleContactTime);
      const auto desiredBasePose = getBasePose(state);
      const auto desiredJointPositions = getJointPositions(state);

      // std::cout << "  desiredBasePose: " << desiredBasePose.transpose() << std::endl;
      // std::cout << "  desiredJointPositions: " << desiredJointPositions.transpose() << std::endl;
      vector3_t referenceFootholdPositionInWorld = kinematicsModel_->footPositionInOriginFrame(leg, desiredBasePose, desiredJointPositions);

      // std::cout << "  referenceFootholdPositionInWorld: " << referenceFootholdPositionInWorld.transpose() << std::endl;

      // Add ZMP offset to the first upcoming foothold.
      // if (contactCount == 0) {
      //   referenceFootholdPositionInWorld += zmpReactiveOffset;
      // }

      // One foothold added per contactPhase
      heuristicFootholds.push_back(referenceFootholdPositionInWorld);

      // Can stop for this leg if we have processed one contact phase after (or extending across) the horizon
      if (contactEndTime > finalTime) {
        break;
      }
    }
    ++contactCount;
  }

  return heuristicFootholds;
}

std::vector<ConvexTerrain> SwingTrajectoryPlanner::selectNominalFootholdTerrain(int leg, const std::vector<ContactTiming>& contactTimings,
                                                                                const std::vector<vector3_t>& heuristicFootholds,
                                                                                const ocs2::TargetTrajectories& targetTrajectories,
                                                                                scalar_t initTime, const comkino_state_t& currentState,
                                                                                scalar_t finalTime,
                                                                                const TerrainModel& terrainModel) const 
{
  // std::cout << "      [SwingTrajectoryPlanner::selectNominalFootholdTerrain]" << std::endl;

  // Will increment the heuristic each time after selecting a nominalFootholdTerrain
  auto heuristicFootholdIt = heuristicFootholds.cbegin();
  std::vector<ConvexTerrain> nominalFootholdTerrain;

  // Nominal foothold is equal to current foothold for legs in contact
  if (startsWithStancePhase(contactTimings)) 
  {
    // std::cout << "        starts with stance phase" << std::endl;
    ConvexTerrain convexTerrain;
    convexTerrain.plane = lastContacts_[leg].second;
    nominalFootholdTerrain.push_back(convexTerrain);
    // std::cout << "        adding last foot position: " << lastContacts_[leg].second.positionInWorld.transpose() << std::endl;
    ++heuristicFootholdIt;  // Skip this heuristic. Use lastContact directly
  }

  // For future contact phases
  // std::cout << "        contact phase loop" << std::endl;
  for (const auto& contactPhase : contactTimings) 
  {
    if (hasStartTime(contactPhase)) 
    {
      // std::cout << "          has start time" << std::endl;
      // std::cout << "          contactPhase.start: " << contactPhase.start << std::endl;

      const scalar_t timeTillContact = contactPhase.start - initTime;
      const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
      // std::cout << "          contactEndTime: " << contactEndTime << std::endl;
      const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

      // Get previous foothold if there was one at this time
      const FootPhase* previousIterationContact = getFootPhaseIfInContact(leg, middleContactTime);

      if (timeTillContact < settings_.previousFootholdTimeDeadzone && previousIterationContact != nullptr) 
      {
        // Simply copy the information out of the previous iteration
        nominalFootholdTerrain.push_back(*previousIterationContact->nominalFootholdConstraint());
        ++heuristicFootholdIt;  // Skip this heuristic. Using the previous terrain instead
      } else 
      {
        // Select the terrain base on the heuristic
        vector3_t referenceFootholdPositionInWorld = *heuristicFootholdIt;

        // std::cout << "          referenceFootholdPositionInWorld (1): " << referenceFootholdPositionInWorld.transpose() << std::endl;

        // Filter w.r.t. previous foothold
        // if (previousIterationContact != nullptr) {
        //   referenceFootholdPositionInWorld =
        //       filterFoothold(referenceFootholdPositionInWorld, previousIterationContact->nominalFootholdLocation());
        // }

        // std::cout << "          referenceFootholdPositionInWorld (2): " << referenceFootholdPositionInWorld.transpose() << std::endl;

        // Kinematic penalty
        const base_coordinate_t basePoseAtTouchdown = getBasePose(targetTrajectories.getDesiredState(contactPhase.start));
        const auto hipPositionInWorldTouchdown = kinematicsModel_->legRootInOriginFrame(leg, basePoseAtTouchdown);
        const auto hipOrientationInWorldTouchdown = kinematicsModel_->orientationLegRootToOriginFrame(leg, basePoseAtTouchdown);
        const base_coordinate_t basePoseAtLiftoff = getBasePose(targetTrajectories.getDesiredState(contactEndTime));
        const auto hipPositionInWorldLiftoff = kinematicsModel_->legRootInOriginFrame(leg, basePoseAtLiftoff);
        const auto hipOrientationInWorldLiftoff = kinematicsModel_->orientationLegRootToOriginFrame(leg, basePoseAtLiftoff);
        ApproximateKinematicsConfig config;
        config.kinematicPenaltyWeight = settings_.legOverExtensionPenalty;
        config.maxLegExtension = settings_.nominalLegExtension;
        
        auto scoringFunction = [&](const vector3_t& footPositionInWorld) 
        {
          return computeKinematicPenalty(footPositionInWorld, hipPositionInWorldTouchdown, hipOrientationInWorldTouchdown, config) +
                 computeKinematicPenalty(footPositionInWorld, hipPositionInWorldLiftoff, hipOrientationInWorldLiftoff, config);
        };

        if (contactPhase.start < finalTime) 
        {
          // std::cout << "          before horizon" << std::endl;
          ConvexTerrain convexTerrain = terrainModel.getConvexTerrainAtPositionInWorld(referenceFootholdPositionInWorld, scoringFunction);
          // std::cout << "          convexTerrain.plane.positionInWorld.transpose(): " << convexTerrain.plane.positionInWorld.transpose() << std::endl;

          nominalFootholdTerrain.push_back(convexTerrain);
          ++heuristicFootholdIt;
        } else 
        {  // After the horizon -> we are only interested in the position and orientation
          // std::cout << "          after horizon" << std::endl;
          ConvexTerrain convexTerrain;
          convexTerrain.plane =
              terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(referenceFootholdPositionInWorld, scoringFunction);
          nominalFootholdTerrain.push_back(convexTerrain);
          // std::cout << "          convexTerrain.plane.positionInWorld.transpose(): " << convexTerrain.plane.positionInWorld.transpose() << std::endl;
          ++heuristicFootholdIt;
        }
      }

      // Can stop for this leg if we have processed one contact phase after (or extending across) the horizon
      if (contactEndTime > finalTime) {
        break;
      }
    }
  }

  return nominalFootholdTerrain;
}

void SwingTrajectoryPlanner::subsampleReferenceTrajectory(const ocs2::TargetTrajectories& targetTrajectories, scalar_t initTime,
                                                          scalar_t finalTime) {
  if (targetTrajectories.empty()) {
    throw std::runtime_error("[SwingTrajectoryPlanner] provided target trajectory cannot be empty.");
  }

  targetTrajectories_.clear();

  // Add first reference
  {
    const auto initInterpIndex = ocs2::LinearInterpolation::timeSegment(initTime, targetTrajectories.timeTrajectory);
    targetTrajectories_.timeTrajectory.push_back(initTime);
    targetTrajectories_.stateTrajectory.push_back(
        ocs2::LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.stateTrajectory));
    targetTrajectories_.inputTrajectory.push_back(
        ocs2::LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.inputTrajectory));
  }

  for (int k = 0; k < targetTrajectories.timeTrajectory.size(); ++k) {
    if (targetTrajectories.timeTrajectory[k] < initTime) {
      continue;  // Drop all samples before init time
    } else if (targetTrajectories.timeTrajectory[k] > finalTime) {
      // Add final time sample. Samples after final time are also kept for touchdowns after the horizon.
      const auto finalInterpIndex = ocs2::LinearInterpolation::timeSegment(finalTime, targetTrajectories.timeTrajectory);
      targetTrajectories_.timeTrajectory.push_back(finalTime);
      targetTrajectories_.stateTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.stateTrajectory));
      targetTrajectories_.inputTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.inputTrajectory));
    }

    // Check if we need to add extra intermediate samples
    while (targetTrajectories_.timeTrajectory.back() + settings_.maximumReferenceSampleTime < targetTrajectories.timeTrajectory[k]) {
      const scalar_t t = targetTrajectories_.timeTrajectory.back() + settings_.maximumReferenceSampleTime;
      const auto interpIndex = ocs2::LinearInterpolation::timeSegment(t, targetTrajectories.timeTrajectory);

      targetTrajectories_.timeTrajectory.push_back(t);
      targetTrajectories_.stateTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(interpIndex, targetTrajectories.stateTrajectory));
      targetTrajectories_.inputTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(interpIndex, targetTrajectories.inputTrajectory));
    }

    // Add the original reference sample
    targetTrajectories_.timeTrajectory.push_back(targetTrajectories.timeTrajectory[k]);
    targetTrajectories_.stateTrajectory.push_back(targetTrajectories.stateTrajectory[k]);
    targetTrajectories_.inputTrajectory.push_back(targetTrajectories.inputTrajectory[k]);
  }
}

void SwingTrajectoryPlanner::adaptJointReferencesWithInverseKinematics(scalar_t finalTime) 
{
  const scalar_t damping = 0.01;  // Quite some damping on the IK to get well conditions references.

  for (int k = 0; k < targetTrajectories_.timeTrajectory.size(); ++k) {
    const scalar_t t = targetTrajectories_.timeTrajectory[k];

    const base_coordinate_t basePose = getBasePose(comkino_state_t(targetTrajectories_.stateTrajectory[k]));
    const vector3_t basePositionInWorld = getPositionInOrigin(basePose);
    const vector3_t eulerXYZ = getOrientation(basePose);

    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) 
    {
      const auto& footPhase = this->getFootPhase(leg, t);

      // Joint positions
      const vector3_t positionBaseToFootInWorldFrame = footPhase.getPositionInWorld(t) - basePositionInWorld;
      const vector3_t positionBaseToFootInBaseFrame = rotateVectorOriginToBase(positionBaseToFootInWorldFrame, eulerXYZ);

      const size_t stateOffset = 2 * BASE_COORDINATE_SIZE + 3 * leg;
      targetTrajectories_.stateTrajectory[k].segment(stateOffset, 3) =
          inverseKinematicsModelPtr_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(leg, positionBaseToFootInBaseFrame);

      // Joint velocities
      auto jointPositions = getJointPositions(targetTrajectories_.stateTrajectory[k]);
      auto baseTwistInBaseFrame = getBaseLocalVelocities(targetTrajectories_.stateTrajectory[k]);

      const vector3_t b_baseToFoot = kinematicsModel_->positionBaseToFootInBaseFrame(leg, jointPositions);
      const vector3_t footVelocityInBaseFrame = rotateVectorOriginToBase(footPhase.getVelocityInWorld(t), eulerXYZ);
      const vector3_t footRelativeVelocityInBaseFrame =
          footVelocityInBaseFrame - getLinearVelocity(baseTwistInBaseFrame) - getAngularVelocity(baseTwistInBaseFrame).cross(b_baseToFoot);

      const size_t inputOffset = 3 * NUM_CONTACT_POINTS + 3 * leg;
      targetTrajectories_.inputTrajectory[k].segment(inputOffset, 3) =
          inverseKinematicsModelPtr_->getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
              leg, footRelativeVelocityInBaseFrame, kinematicsModel_->baseToFootJacobianBlockInBaseFrame(leg, jointPositions), damping);
    }

    // Can stop adaptation as soon as we have processed a point beyond the horizon.
    if (t > finalTime) {
      break;
    }
  }
}

void SwingTrajectoryPlanner::updateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition,
                                               const TerrainModel& terrainModel) {
  // Get orientation from terrain model, position from the kinematics
  auto lastContactTerrain = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(currentFootPosition);
  lastContactTerrain.positionInWorld = currentFootPosition;
  lastContacts_[leg] = {expectedLiftOff, lastContactTerrain};
}

SwingPhase::SwingProfile SwingTrajectoryPlanner::getDefaultSwingProfile() const 
{
  // std::cout << "[SwingTrajectoryPlanner::getDefaultSwingProfile]" << std::endl;

  SwingPhase::SwingProfile defaultSwingProfile;
  defaultSwingProfile.sdfMidswingMargin = settings_.sdfMidswingMargin;
  defaultSwingProfile.maxSwingHeightAdaptation = 2.0 * settings_.swingHeight;

  SwingPhase::SwingProfile::Node midPoint;
  midPoint.phase = 0.5;
  midPoint.swingHeight = settings_.swingHeight;
  midPoint.normalVelocity = 0.0;
  midPoint.tangentialProgress = 0.6;
  midPoint.tangentialVelocityFactor = 2.0;
  
  // std::cout << "midPoint.swingHeight: " << midPoint.swingHeight << std::endl;  
  
  defaultSwingProfile.nodes.push_back(midPoint);
  return defaultSwingProfile;
}

scalar_t SwingTrajectoryPlanner::getContactEndTime(const ContactTiming& contactPhase, scalar_t finalTime) const 
{
  return hasEndTime(contactPhase) ? contactPhase.end : std::max(finalTime + settings_.referenceExtensionAfterHorizon, contactPhase.start);
}

const FootPhase* SwingTrajectoryPlanner::getFootPhaseIfInContact(size_t leg, scalar_t time) const {
  const FootPhase* previousIterationContact = nullptr;
  if (!feetNormalTrajectories_[leg].empty()) {
    const auto& footPhase = getFootPhase(leg, time);
    if (footPhase.contactFlag()) {
      previousIterationContact = &footPhase;
    }
  }
  return previousIterationContact;
}

vector3_t SwingTrajectoryPlanner::filterFoothold(const vector3_t& newFoothold, const vector3_t& previousFoothold) const {
  // Apply Position deadzone and low pass filter
  if ((newFoothold - previousFoothold).norm() < settings_.previousFootholdDeadzone) {
    return previousFoothold;
  } else {
    // low pass filter
    const scalar_t lambda = settings_.previousFootholdFactor;
    return lambda * previousFoothold + (1.0 - lambda) * newFoothold;
  }
}

SwingTrajectoryPlannerSettings loadSwingTrajectorySettings(const std::string& filename, bool verbose) {
  SwingTrajectoryPlannerSettings settings{};

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const std::string prefix{"model_settings.swing_trajectory_settings."};

  if (verbose) {
    std::cerr << "\n #### Swing trajectory Settings:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  ocs2::loadData::loadPtreeValue(pt, settings.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingHeight, prefix + "swingHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.errorGain, prefix + "errorGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTimeScale, prefix + "swingTimeScale", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.sdfMidswingMargin, prefix + "sdfMidswingMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.terrainMargin, prefix + "terrainMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdFactor, prefix + "previousFootholdFactor", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdDeadzone, prefix + "previousFootholdDeadzone", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdTimeDeadzone, prefix + "previousFootholdTimeDeadzone", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.invertedPendulumHeight, prefix + "invertedPendulumHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.nominalLegExtension, prefix + "nominalLegExtension", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.legOverExtensionPenalty, prefix + "legOverExtensionPenalty", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.referenceExtensionAfterHorizon, prefix + "referenceExtensionAfterHorizon", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.maximumReferenceSampleTime, prefix + "maximumReferenceSampleTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTrajectoryFromReference, prefix + "swingTrajectoryFromReference", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.setModeScheduleFromGait, prefix + "setModeScheduleFromGait", verbose);

  if (verbose) {
    std::cerr << " #### ==================================================" << std::endl;
  }

  return settings;
}

}  // namespace switched_model

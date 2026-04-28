//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(std::unique_ptr<GaitSchedule> gaitSchedule,
                                                                   std::unique_ptr<SwingTrajectoryPlanner> swingTrajectory,
                                                                   std::unique_ptr<TerrainModel> terrainModel)
    : gaitSchedule_(std::move(gaitSchedule)), swingTrajectoryPtr_(std::move(swingTrajectory)), terrainModel_(std::move(terrainModel)) {}

contact_flag_t SwitchedModelModeScheduleManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelModeScheduleManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                        ocs2::TargetTrajectories& targetTrajectories, ocs2::ModeSchedule& modeSchedule) 
{
  // std::cout << "[SwitchedModelModeScheduleManager::modifyReferences] started" << std::endl;

  // std::cout << "    terrainModel_" << std::endl;


  const auto timeHorizon = finalTime - initTime;

  // std::cout << "    initTime: " << initTime << std::endl;
  // std::cout << "    finalTime: " << finalTime << std::endl;

  // std::cout << "    timeHorizon: " << timeHorizon << std::endl;

  // std::cout << "    swingTrajectoryPtr_->settings().referenceExtensionAfterHorizon: " << 
  //                   swingTrajectoryPtr_->settings().referenceExtensionAfterHorizon << std::endl;

  // std::cout << "    (PRE) modeSchedule: " << modeSchedule << std::endl;

  // Max: don't want gait schedule to overwrite mode schedule, commenting out for now. 
  if (swingTrajectoryPtr_->settings().setModeScheduleFromGait) 
  {
    // std::cout << "loading mode schedule from gait" << std::endl;
    auto lockedGaitSchedulePtr = gaitSchedule_.lock();
    lockedGaitSchedulePtr->advanceToTime(initTime);
    modeSchedule = lockedGaitSchedulePtr->getModeSchedule(timeHorizon + swingTrajectoryPtr_->settings().referenceExtensionAfterHorizon);
  }

  // std::cout << "    (POST) modeSchedule: " << modeSchedule << std::endl;

  // Transfer terrain ownership if a new terrain is available
  std::unique_ptr<TerrainModel> newTerrain;
  terrainModel_.swap(newTerrain);  // Thread-safe swap with lockable Terrain
  if (newTerrain) 
  {
    // std::cout << "    new terrain" << std::endl;
    swingTrajectoryPtr_->updateTerrain(std::move(newTerrain));
  }


  // std::cout << "    swingTrajectoryPtr_->getTerrainModel().getAllConvexTerrains().size(): " << swingTrajectoryPtr_->getTerrainModel().getAllConvexTerrains().size() << std::endl;
  
  // std::cout << "    pre-swingTrajectoryPtr_->updateSwingMotions" << std::endl;    
  // Prepare swing motions
  swingTrajectoryPtr_->updateSwingMotions(initTime, finalTime, initState, targetTrajectories, modeSchedule);
  // std::cout << "    post-swingTrajectoryPtr_->updateSwingMotions" << std::endl;  

  // std::cout << "[SwitchedModelModeScheduleManager::modifyReferences] finished" << std::endl;

}

}  // namespace switched_model

//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/InverseKinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

#include "ocs2_go2_models/FrameDeclaration.h"

namespace go2 {

enum class UnitreeModel { Go2 };

std::string toString(UnitreeModel model);

UnitreeModel stringToUnitreeModel(const std::string& name);

std::string getUrdfPath(UnitreeModel model);
std::string getUrdfString(UnitreeModel model);
std::string getUrdfString(const std::string & urdfPath);

std::unique_ptr<switched_model::InverseKinematicsModelBase> getGo2InverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getGo2Kinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getGo2KinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getGo2ComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getGo2ComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf);

}  // namespace go2

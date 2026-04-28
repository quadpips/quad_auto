//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/FrameDeclaration.h>

namespace anymal {

std::shared_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, 
                                                                            const std::string& taskFolder); // const std::string& envFile 

std::shared_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, 
                                                                            const std::string& taskFilePath,
                                                                            const std::string& frameFilePath); // const std::string& envFile 

std::shared_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                            switched_model::QuadrupedInterface::Settings settings,
                                                                            const FrameDeclaration& frameDeclaration); // const std::string& envFile 

}  // namespace anymal

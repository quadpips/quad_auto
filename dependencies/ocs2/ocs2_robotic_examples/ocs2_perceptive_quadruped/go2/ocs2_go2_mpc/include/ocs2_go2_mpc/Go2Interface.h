//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

#include <ocs2_go2_models/Go2Models.h>
#include <ocs2_go2_models/FrameDeclaration.h>

namespace go2 {

std::shared_ptr<switched_model::CustomQuadrupedInterface> getGo2Interface(const std::string& urdf, 
                                                                            const std::string& taskFolder); // const std::string& envFile 

std::shared_ptr<switched_model::CustomQuadrupedInterface> getGo2Interface(const std::string& urdf, 
                                                                            const std::string& taskFilePath,
                                                                            const std::string& frameFilePath); // const std::string& envFile 

std::shared_ptr<switched_model::CustomQuadrupedInterface> getGo2Interface(const std::string& urdf,
                                                                            switched_model::CustomQuadrupedInterface::Settings settings,
                                                                            const FrameDeclaration& frameDeclaration); // const std::string& envFile 

}  // namespace go2

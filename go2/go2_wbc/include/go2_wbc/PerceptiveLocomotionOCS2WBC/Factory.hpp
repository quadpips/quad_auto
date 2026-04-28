#pragma once

#include <memory>
#include <string>
#include <vector>

#include "go2_wbc/PerceptiveLocomotionOCS2WBC/HqpWbc.hpp"
#include "go2_wbc/PerceptiveLocomotionOCS2WBC/SqpWbc.hpp"
#include <ocs2_go2_mpc/Go2Interface.h>

#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace switched_model {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames);

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames);

}  // namespace switched_model

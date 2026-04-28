#pragma once

#include <legged_common/control/ctrl_utils/QuadrupedParameters.h>
#include <legged_common/control/ctrl_utils/HighCmd.h>
#include "go2_system/controllers/auxillary/LegController.h"
#include <go2_estimators/estimators/StateEstimatorContainer.h>

namespace legged_software {
namespace go2_system {

using namespace legged_common;
using namespace go2_estimators;

/**
 *
 */
struct ControlFSMData 
{
  StateEstimatorContainer* _stateEstimator;
  LegController* _legController;
  HighCmdCustom* _highcmd;
  QuadrupedParameters* userParameters;
};


} // namespace go2_system
} // namespace legged_software

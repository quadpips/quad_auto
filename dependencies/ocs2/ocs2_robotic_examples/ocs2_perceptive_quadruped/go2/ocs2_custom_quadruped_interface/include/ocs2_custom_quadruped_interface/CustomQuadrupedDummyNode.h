//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "CustomQuadrupedInterface.h"

namespace switched_model {

void customQuadrupedDummyNode(const rclcpp::Node::SharedPtr &node, const CustomQuadrupedInterface& quadrupedInterface, const ocs2::RolloutBase* rolloutPtr,
                        double mrtDesiredFrequency, double mpcDesiredFrequency);
}

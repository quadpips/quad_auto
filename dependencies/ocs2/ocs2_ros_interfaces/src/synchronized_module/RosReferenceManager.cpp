/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include "rclcpp/rclcpp.hpp"

// MPC messages
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RosReferenceManager::RosReferenceManager(
    std::string topicPrefix,
    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
      topicPrefix_(std::move(topicPrefix)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::subscribe(const rclcpp::Node::SharedPtr& node) {
  node_ = node;
  // ModeSchedule
  auto modeScheduleCallback = [this](const ocs2_msgs::msg::ModeSchedule& msg) {
    auto modeSchedule = ros_msg_conversions::readModeScheduleMsg(msg);
    referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
  };
  modeScheduleSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::ModeSchedule>(
          topicPrefix_ + "_mode_schedule", 1, modeScheduleCallback);

  // TargetTrajectories
  auto targetTrajectoriesCallback =
      [this](const ocs2_msgs::msg::MpcTargetTrajectories& msg) 
      {
        auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(msg);
        referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));

        // auto tmpTargetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(msg);
        // std::string stateStr = "Received new target trajectories (size: " + 
        //                        std::to_string(tmpTargetTrajectories.timeTrajectory.size()) + "):";
        // for (int i = 0; i < tmpTargetTrajectories.timeTrajectory.size(); i++) 
        // {
        //   stateStr += "\n";
        //   stateStr += "  time: " + std::to_string(tmpTargetTrajectories.timeTrajectory[i]) + "\n";
        //   stateStr += "     x: " + std::to_string(tmpTargetTrajectories.stateTrajectory[i][0]) + ", " + 
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][1]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][2]) + ", " + 
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][3]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][4]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][5]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][6]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][7]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][8]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][9]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][10]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][11]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][12]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][13]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][14]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][15]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][16]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][17]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][18]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][19]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][20]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][21]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][22]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.stateTrajectory[i][23]);
        //   stateStr += "     u: " + std::to_string(tmpTargetTrajectories.inputTrajectory[i][0]) + ", " + 
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][1]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][2]) + ", " + 
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][3]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][4]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][5]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][6]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][7]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][8]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][9]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][10]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][11]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][12]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][13]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][14]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][15]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][16]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][17]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][18]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][19]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][20]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][21]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][22]) + ", " +
        //                           std::to_string(tmpTargetTrajectories.inputTrajectory[i][23]);
        // }

        // RCLCPP_INFO(node_->get_logger(), "%s", stateStr.c_str());
      };
  targetTrajectoriesSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
          topicPrefix_ + "_mpc_target", 1, targetTrajectoriesCallback);
}

}  // namespace ocs2

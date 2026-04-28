/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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


// #include <ros/init.h>
// #include <ros/node_handle.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// #include <ocs2_go2_mpc/Go2Interface.h>

#include <ocs2_custom_quadruped_interface/CustomTerrainReceiver.h>
// #include <grid_map_ros/GridMapRosConverter.hpp>

#include <ocs2_go2_mpc/Go2Interface.h>
#include <mmp_quadruped/visualization/MultiModalVisualizer.h>
#include <mmp_quadruped/planning/MultiModalPlanner.h>

// using namespace ocs2;
// using namespace mmp;

using namespace switched_model;
using namespace mmp;

int main(int argc, char** argv) 
{
  const std::string robotName = "go2";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_mmp",
                                                              rclcpp::NodeOptions()
                                                              .allow_undeclared_parameters(true)
                                                              .automatically_declare_parameters_from_overrides(true));


  RCLCPP_INFO_STREAM(node->get_logger(), "Starting " << robotName << " Multi-Modal Planner Node");

  RCLCPP_INFO_STREAM(node->get_logger(), "Loading string parameters...");

  // Get node parameters
  std::string hyperparametersFile, taskFile, dummyTaskFile, sqpFile, frameFile, 
              dummyFrameFile, urdfFile, dummyUrdfFile, envFile, gaitCommandFile, gait;
  node->get_parameter("hyperparametersFile", hyperparametersFile);
  node->get_parameter("mmpSqpFile", sqpFile);
  node->get_parameter("mmpTaskFile", taskFile);
  node->get_parameter("mmpFrameFile", frameFile);
  node->get_parameter("mmpUrdfFile", urdfFile);

  node->get_parameter("dummyTaskFile", dummyTaskFile);
  node->get_parameter("dummyFrameFile", dummyFrameFile);
  node->get_parameter("dummyUrdfFile", dummyUrdfFile);

  node->get_parameter("envFile", envFile);
  node->get_parameter("gaitCommandFile", gaitCommandFile);

  RCLCPP_INFO_STREAM(node->get_logger(), "...Done loading string parameters!");

  RCLCPP_INFO_STREAM(node->get_logger(), "Loading bool parameters...");

  node->get_parameter("gait", gait);

  RCLCPP_INFO_STREAM(node->get_logger(), "...Done loading bool parameters!");

  RCLCPP_INFO_STREAM(node->get_logger(), "Loading Go2+Dummy interface ...");

  std::string urdfString = go2::getUrdfString(urdfFile);
  std::string dummyUrdfString = go2::getUrdfString(dummyUrdfFile);

  auto go2Interface = go2::getGo2Interface(urdfString, taskFile, frameFile); // , envFile
  auto dummyInterface = go2::getGo2Interface(dummyUrdfString, dummyTaskFile, dummyFrameFile); // for vis // , envFile

  RCLCPP_INFO_STREAM(node->get_logger(), "... Done loading Go2+Dummy interface!");

  RCLCPP_INFO_STREAM(node->get_logger(), "Adding terrain receiver ...");

  // Add terrain receiver
  auto solverModules = go2Interface->getSynchronizedModules();

  // Terrain Receiver
  auto customTerrainReceiver = std::make_shared<CustomTerrainReceiverSynchronizedModule>(
                      go2Interface->getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(),
                      node,
                      false);
  solverModules.push_back(customTerrainReceiver);

  RCLCPP_INFO_STREAM(node->get_logger(), "... Done adding terrain receiver!");

  RCLCPP_INFO_STREAM(node->get_logger(), "Adding visualizers ...");

  // Visualization
  std::shared_ptr<Go2Visualizer> go2Visualizer(new Go2Visualizer(dummyInterface->getKinematicModel(), 
                                                                          dummyInterface->getJointNames(), 
                                                                          dummyInterface->getBaseName(),
                                                                          node));


  std::shared_ptr<MultiModalVisualizer> multiModalVisualizer(new MultiModalVisualizer(node, 
                                                                                      go2Visualizer)); 

  RCLCPP_INFO_STREAM(node->get_logger(), "... Done adding visualizers!");

  // SqpSolver
  ocs2::sqp::Settings sqpSettings = ocs2::sqp::loadSettings(sqpFile); // const ocs2::log::Settings 
  std::shared_ptr<ocs2::SqpSolver> solverPtr = std::make_shared<ocs2::SqpSolver>(sqpSettings, go2Interface->getOptimalControlProblem(), go2Interface->getInitializer());
  solverPtr->setReferenceManager(go2Interface->getReferenceManagerPtr());
  solverPtr->setSynchronizedModules(solverModules);

  RCLCPP_INFO_STREAM(node->get_logger(), "Adding planner ...");

  // Multi-modal planner
  MultiModalPlanner * planner = new MultiModalPlanner(node, multiModalVisualizer, 
                                                      go2Interface, taskFile, sqpFile, 
                                                      hyperparametersFile, envFile,
                                                      gaitCommandFile, gait); 

  RCLCPP_INFO_STREAM(node->get_logger(), "... Done adding planner!");
  
  scalar_t dummyTimeHorizon = 0.01;

  bool planResult = false;
  int numRetries = 0;
  int maxNumRetries = 3;
  while (rclcpp::ok() && !planner->reachedGlobalGoal())
  {
    ///////////////////////////////////
    // Run dummy problem for terrain //
    ///////////////////////////////////
    RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for planner ...");
    while (rclcpp::ok() && !planner->readyToPlan())
    {
      // run dummy problem to get a preSolverRun call in to update the terrain
      solverPtr->run(0.0, go2Interface->getInitialState(), dummyTimeHorizon);

      rclcpp::spin_some(node); // need because we are receiving callbacks
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "... Planner is ready!");

    /////////////////
    // Run planner //
    /////////////////
    RCLCPP_INFO_STREAM(node->get_logger(), "Planning ...");

    planResult = planner->runPlanner(solverPtr);

    RCLCPP_INFO_STREAM(node->get_logger(), "... Done planning!");

    rclcpp::spin_some(node); // need because we are receiving callbacks

    if (planResult)
    {
      ////////////////////////
      // Prepare trajectory //
      ////////////////////////

      RCLCPP_INFO_STREAM(node->get_logger(), "#=======================================================================#");
      RCLCPP_INFO_STREAM(node->get_logger(), "# II - PREPARING REFERENCE TRAJECTORY                                   #");
      RCLCPP_INFO_STREAM(node->get_logger(), "#=======================================================================#");

      planner->prepareReferenceTrajectory(solverPtr);

      rclcpp::spin_some(node); // need because we are receiving callbacks

      ////////////////
      // Track plan //
      ////////////////
      if (planner->generatedPlan())
      {
        numRetries = 0; // reset retries
        
        RCLCPP_INFO_STREAM(node->get_logger(), "#=======================================================================#");
        RCLCPP_INFO_STREAM(node->get_logger(), "# III - PUBLISHING REFERENCE TRAJECTORY                                 #");
        RCLCPP_INFO_STREAM(node->get_logger(), "#=======================================================================#");

        planner->publishReferenceTrajectory();  
        
        rclcpp::spin_some(node); // need because we are receiving callbacks      

        // RCLCPP_INFO_STREAM(node->get_logger(), "Tracking ...");

        // planner->trackPlan(solverPtr, planResult);
        RCLCPP_INFO_STREAM(node->get_logger(), "#=======================================================================#");
        RCLCPP_INFO_STREAM(node->get_logger(), "# IV - TRACKING REFERENCE TRAJECTORY                                    #");
        RCLCPP_INFO_STREAM(node->get_logger(), "#=======================================================================#");

        planner->monitorReferenceTrajectory();        

        // RCLCPP_INFO_STREAM(node->get_logger(), "... Done tracking!");
      } else
      {
        RCLCPP_WARN_STREAM(node->get_logger(), "No plan was generated, skipping tracking!");
      }
    } else
    {
      // throw std::runtime_error("[LeggedRobotMmpNode]: Planning failed!");
      if (numRetries < maxNumRetries)
      {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Planning failed! Retrying ...");
        numRetries++;
      } else
      {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Planning failed! Maximum number of retries reached!");
        throw std::runtime_error("[LeggedRobotMmpNode]: Planning failed!");
      }
    }

    //////////////////////////
    // Check if we are done //
    //////////////////////////
    planner->checkGlobalGoalStatus();

    RCLCPP_INFO_STREAM(node->get_logger(), "Clearing ...");

    /////////////////////// 
    // Clear for re-plan //
    ///////////////////////
    planner->clear();

    RCLCPP_INFO_STREAM(node->get_logger(), "... Done clearing!");
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "All done!");
  
  // delete planner;

  // Successful exit
  return 0;
}

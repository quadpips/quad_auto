//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_mpc/AnymalInterface.h"

// #include "rclcpp/rclcpp.hpp"
// #include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>

namespace anymal {

std::shared_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, 
                                                                          const std::string& taskFolder) // const std::string& envFile  
{
  // std::cerr << "Loading task file from: " << taskFolder << std::endl;

  return getAnymalInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                            frameDeclarationFromFile(taskFolder + "/frame_declaration.info")); // const std::string& envFile
}

std::shared_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, 
                                                                          const std::string& taskFilePath,
                                                                          const std::string& frameFilePath) // const std::string& envFile
{
  // std::cerr << "[getAnymalInterface(urdf, task, frame)] " << std::endl;
  // std::cerr << "Loading task file from: " << taskFilePath << std::endl;
  // std::cerr << "Loading frame file from: " << frameFilePath << std::endl;

  return getAnymalInterface(urdf, switched_model::loadQuadrupedSettings(taskFilePath),
                          frameDeclarationFromFile(frameFilePath)); // , envFile            
}

std::shared_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                          switched_model::QuadrupedInterface::Settings settings,
                                                                          const FrameDeclaration& frameDeclaration) // , const std::string& envFile  
{
  // std::cerr << "[getAnymalInterface(urdf, settings, frameDeclaration)] " << std::endl;

  std::shared_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};

  if (settings.modelSettings_.analyticalInverseKinematics_) 
  {
    // std::cerr << "Creating analytical inverse kinematics ..." << std::endl;
    invKin = getAnymalInverseKinematics(frameDeclaration, urdf);
    // std::cerr << "Created analytical inverse kinematics." << std::endl;
  }
  
  // std::cerr << "Creating kinematics model ..." << std::endl;
  auto kin = getAnymalKinematics(frameDeclaration, urdf);
  // std::cerr << "Creating kinematics AD model ..." << std::endl;
  auto kinAd = getAnymalKinematicsAd(frameDeclaration, urdf);
  // std::cerr << "Creating COM model ..." << std::endl;
  auto com = getAnymalComModel(frameDeclaration, urdf);
  // std::cerr << "Creating COM AD model ..." << std::endl;
  auto comAd = getAnymalComModelAd(frameDeclaration, urdf);
  // std::cerr << "Getting joint names ..." << std::endl;
  auto jointNames = getJointNames(frameDeclaration);
  // std::cerr << "Getting base name ..." << std::endl;
  auto baseName = frameDeclaration.root;

  return std::shared_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedPointfootInterface(
      *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName))); // , envFile 
}

}  // namespace anymal

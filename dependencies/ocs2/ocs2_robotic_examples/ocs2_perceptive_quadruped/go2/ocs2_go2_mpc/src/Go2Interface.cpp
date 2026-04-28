//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_go2_mpc/Go2Interface.h"

// #include "rclcpp/rclcpp.hpp"
// #include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedPointfootInterface.h>

namespace go2 {

std::shared_ptr<switched_model::CustomQuadrupedInterface> getGo2Interface(const std::string& urdf, 
                                                                          const std::string& taskFolder) // const std::string& envFile  
{
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  return getGo2Interface(urdf, switched_model::loadCustomQuadrupedSettings(taskFolder + "/task.info"),
                            frameDeclarationFromFile(taskFolder + "/frame_declaration.info")); // const std::string& envFile
}

std::shared_ptr<switched_model::CustomQuadrupedInterface> getGo2Interface(const std::string& urdf, 
                                                                          const std::string& taskFilePath,
                                                                          const std::string& frameFilePath) // const std::string& envFile
{
  std::cerr << "Loading task file from: " << taskFilePath << std::endl;
  std::cerr << "Loading frame file from: " << frameFilePath << std::endl;

  return getGo2Interface(urdf, switched_model::loadCustomQuadrupedSettings(taskFilePath),
                          frameDeclarationFromFile(frameFilePath)); // , envFile            
}

std::shared_ptr<switched_model::CustomQuadrupedInterface> getGo2Interface(const std::string& urdf,
                                                                          switched_model::CustomQuadrupedInterface::Settings settings,
                                                                          const FrameDeclaration& frameDeclaration) // , const std::string& envFile  
{
  std::shared_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};

  if (settings.modelSettings_.analyticalInverseKinematics_) 
  {
    invKin = getGo2InverseKinematics(frameDeclaration, urdf);
  }
  
  auto kin = getGo2Kinematics(frameDeclaration, urdf);
  auto kinAd = getGo2KinematicsAd(frameDeclaration, urdf);
  auto com = getGo2ComModel(frameDeclaration, urdf);
  auto comAd = getGo2ComModelAd(frameDeclaration, urdf);
  auto jointNames = getJointNames(frameDeclaration);
  auto baseName = frameDeclaration.root;

  return std::shared_ptr<switched_model::CustomQuadrupedInterface>(new switched_model::CustomQuadrupedPointfootInterface(
      *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName))); // , envFile 
}

}  // namespace go2

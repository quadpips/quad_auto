//
// Created by rgrandia on 22.09.20.
//

#include <ocs2_go2_models/Go2Models.h>

#include <unordered_map>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_go2_models/QuadrupedCom.h>
#include <ocs2_go2_models/QuadrupedInverseKinematics.h>
#include <ocs2_go2_models/QuadrupedKinematics.h>

#include <ocs2_go2_models/package_path.h>

namespace go2 {

std::string toString(UnitreeModel model) {
  static const std::unordered_map<UnitreeModel, std::string> map{{UnitreeModel::Go2, "go2"}};
  return map.at(model);
}

UnitreeModel stringToUnitreeModel(const std::string& name) {
  static const std::unordered_map<std::string, UnitreeModel> map{{"go2", UnitreeModel::Go2}};
  return map.at(name);
}

std::string getUrdfPath(UnitreeModel model) {
  switch (model) {
    case UnitreeModel::Go2:
      return getPath() + "/urdf/go2.urdf";
    default:
      throw std::runtime_error("[Go2Models] no default urdf available");
  }
}

std::string getUrdfString(const std::string & urdfPath) {
  std::ifstream stream(urdfPath.c_str());
  if (!stream) {
    throw std::runtime_error("File " + urdfPath + " does not exist");
  }

  std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  return xml_str;
}

std::string getUrdfString(UnitreeModel model) {
  const auto path = getUrdfPath(model);
  std::ifstream stream(path.c_str());
  if (!stream) {
    throw std::runtime_error("File " + path + " does not exist");
  }

  std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  return xml_str;
}

std::unique_ptr<switched_model::InverseKinematicsModelBase> getGo2InverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf) {
  return std::unique_ptr<switched_model::InverseKinematicsModelBase>(
      new QuadrupedInverseKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getGo2Kinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf) {
  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(
      new QuadrupedKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getGo2KinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf) {
  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(
      new QuadrupedKinematicsAd(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getGo2ComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf) {
  return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(
      new QuadrupedCom(frameDeclaration, createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getGo2ComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf) {
  return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(
      new QuadrupedComAd(frameDeclaration, createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

}  // namespace go2

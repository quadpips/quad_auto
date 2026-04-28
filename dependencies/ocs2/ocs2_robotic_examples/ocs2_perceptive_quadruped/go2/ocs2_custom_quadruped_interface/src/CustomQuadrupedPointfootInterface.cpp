//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_custom_quadruped_interface/CustomQuadrupedPointfootInterface.h"

#include <ocs2_ddp/ContinuousTimeLqr.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

CustomQuadrupedPointfootInterface::CustomQuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel,
                                                         const InverseKinematicsModelBase* inverseKinematics, Settings settings,
                                                         std::vector<std::string> jointNames, std::string baseName)
    : CustomQuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, inverseKinematics, std::move(settings),
                                std::move(jointNames), std::move(baseName))
{
  // nominal values
  const auto stanceFlags = switched_model::constantFeetArray(true);
  const auto uSystemForWeightCompensation = weightCompensatingInputs(getComModel(), stanceFlags, switched_model::vector3_t::Zero());
  const auto jointTorquesForWeightCompensation = torqueApproximation(
      getJointPositions(getInitialState()), toArray<scalar_t>(uSystemForWeightCompensation.head<3 * NUM_CONTACT_POINTS>()), kinematicModel);

  std::cout << "getComModel().totalMass(): " << getComModel().totalMass() << std::endl;

  std::cout << "uSystemForWeightCompensation: " << uSystemForWeightCompensation.transpose() << std::endl;

  problemPtr_->preComputationPtr = createPrecomputation();

  // Cost terms
  problemPtr_->costPtr->add("MotionTrackingCost", createMotionTrackingCost());
  problemPtr_->stateCostPtr->add("FootPlacementCost", createFootPlacementCost());
  // problemPtr_->stateCostPtr->add("CollisionAvoidanceCost", createCollisionAvoidanceCost());
  problemPtr_->costPtr->add("JointLimitCost", createJointLimitsSoftConstraint());
  problemPtr_->costPtr->add("TorqueLimitCost", createTorqueLimitsSoftConstraint(jointTorquesForWeightCompensation));
  problemPtr_->costPtr->add("FrictionCones", createFrictionConeCost());

  // Dynamics
  problemPtr_->dynamicsPtr = createDynamics();

  // Per leg terms
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
  {
    const auto& footName = feetNames[i];
    problemPtr_->equalityConstraintPtr->add(footName + "_ZeroForce", createZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_EENormal", createFootNormalConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_EEVel", createEndEffectorVelocityConstraint(i));
  }

  // Initialize cost to be able to query it
  ocs2::TargetTrajectories targetTrajectories({0.0}, {getInitialState()}, {uSystemForWeightCompensation});
  problemPtr_->targetTrajectoriesPtr = &targetTrajectories;

  getSwitchedModelModeScheduleManagerPtr()->setTargetTrajectories(targetTrajectories);
  getSwitchedModelModeScheduleManagerPtr()->preSolverRun(0.0, 0.01, getInitialState()); // final time hard-coded here
  auto lqrSolution = ocs2::continuous_time_lqr::solve(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);

  // std::cout << "lqrSolution.valueFunction: " << lqrSolution.valueFunction << std::endl;

  // lqrSolution.valueFunction *= 10.0;
  // problemPtr_->finalCostPtr->add("lqr_terminal_cost", createMotionTrackingTerminalCost(lqrSolution.valueFunction));
  // problemPtr_->finalCostPtr->add("terminal_cost", createMotionTrackingTerminalCost());

  // Store cost approximation at nominal state input
  auto& preComputation = *problemPtr_->preComputationPtr;
  constexpr auto request = ocs2::Request::Cost + ocs2::Request::SoftConstraint + ocs2::Request::Approximation;
  preComputation.request(request, 0.0, getInitialState(), uSystemForWeightCompensation);
  nominalCostApproximation_ = ocs2::approximateCost(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);

  // Reset, the target trajectories pointed to are local
  problemPtr_->targetTrajectoriesPtr = nullptr;

  initializerPtr_.reset(new ComKinoInitializer(getComModel(), *getSwitchedModelModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings()));
}

}  // namespace switched_model

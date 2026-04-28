//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

#include <ocs2_ddp/ContinuousTimeLqr.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel,
                                                         const InverseKinematicsModelBase* inverseKinematics, Settings settings,
                                                         std::vector<std::string> jointNames, std::string baseName)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, inverseKinematics, std::move(settings),
                                std::move(jointNames), std::move(baseName))
{
  std::cerr << "[QuadrupedPointfootInterface constructor] " << std::endl;
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

  // getSwitchedModelModeScheduleManagerPtr()->setTargetTrajectories(targetTrajectories);
  // getSwitchedModelModeScheduleManagerPtr()->preSolverRun(0.0, 0.01, getInitialState()); // final time hard-coded here
  // auto lqrSolution = ocs2::continuous_time_lqr::solve(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);

  getSwitchedModelModeScheduleManagerPtr()->setTargetTrajectories(targetTrajectories);
  getSwitchedModelModeScheduleManagerPtr()->preSolverRun(0.0, 1.0, getInitialState());
  auto lqrSolution = ocs2::continuous_time_lqr::solve(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);
  lqrSolution.valueFunction *= 100.0;
  problemPtr_->finalCostPtr->add("lqr_terminal_cost", createMotionTrackingTerminalCost(lqrSolution.valueFunction));
  // std::cout << "LQR Value Function at initial state: " << lqrSolution.valueFunction << std::endl;
  // std::cout << "LQR value function size: " << lqrSolution.valueFunction.rows() << " x " << lqrSolution.valueFunction.cols() << std::endl;

  // // Not an improvement
  // scalar_t weight = 10.0;
  // matrix_t Qf = matrix_t::Zero(24, 24);
  // Qf(0, 0)    = weight * 100.0; // torso roll
  // Qf(1, 1)    = weight * 300.0; // torso pitch
  // Qf(2, 2)    = weight * 300.0; // torso yaw
  // Qf(3, 3)    = weight * 1000.0; // com x
  // Qf(4, 4)    = weight * 1000.0; // com y
  // Qf(5, 5)    = weight * 1500.0; // com z
  // Qf(6, 6)    = weight * 10.0; // torso angular vel x
  // Qf(7, 7)    = weight * 30.0; // torso angular vel y
  // Qf(8, 8)    = weight * 30.0; // torso angular vel z
  // Qf(9, 9)    = weight * 15.0;  // base vx
  // Qf(10, 10)  = weight * 15.0;  // base vy
  // Qf(11, 11)  = weight * 30.0;  // base vz
  // Qf(12, 12)  = weight * 2.0;  // front left hip
  // Qf(13, 13)  = weight * 2.0;  // front left knee
  // Qf(14, 14)  = weight * 1.0;  // front left ankle
  // Qf(15, 15)  = weight * 2.0;  // front right hip
  // Qf(16, 16)  = weight * 2.0;  // front right knee
  // Qf(17, 17)  = weight * 1.0;  // front right ankle
  // Qf(18, 18)  = weight * 2.0;  // rear left hip
  // Qf(19, 19)  = weight * 2.0;  // rear left knee
  // Qf(20, 20)  = weight * 1.0;  // rear left ankle
  // Qf(21, 21)  = weight * 2.0;  // rear right hip
  // Qf(22, 22)  = weight * 2.0;  // rear right knee
  // Qf(23, 23)  = weight * 1.0;  // rear right ankle
  // problemPtr_->finalCostPtr->add("lqr_terminal_cost", createMotionTrackingTerminalCost(Qf));

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

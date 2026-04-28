#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <vector>
#include <string>

#include <ocs2_go2_models/QuadrupedCom.h>
#include <ocs2_go2_models/QuadrupedKinematics.h>
#include <go2_wbc/PerceptiveLocomotionOCS2WBC/Task.hpp>

namespace switched_model {

struct JointCommand {
        std::string joint_name = "";
        scalar_t desired_position = 0.0;
        scalar_t desired_velocity = 0.0;
        scalar_t kp = 0.0;
        scalar_t kd = 0.0;
        scalar_t torque_ff = 0.0;
};

class WbcBase {
   public:
    WbcBase(const std::string &configFile, const std::string &urdfString,
            const switched_model::ComModelBase<scalar_t> &comModel,
            const switched_model::KinematicsModelBase<scalar_t> &kinematics, const std::string &configPrefix);

    virtual ~WbcBase() = default;

    virtual std::vector<JointCommand> getCommandMessage(scalar_t currentTime, const vector_t &currentState,
                                                                const vector_t &currentInput, const size_t currentMode,
                                                                const vector_t &desiredState, const vector_t &desiredInput,
                                                                const size_t desiredMode,
                                                                const vector_t &desiredJointAcceleration,
                                                                int & flag) = 0;

   protected:
    Task createDynamicsTask();
    Task createContactForceTask();
    Task createStanceFootNoMotionTask();
    Task createTorqueLimitTask();

    Task createBaseAccelerationTask(const vector_t &stateCurrent, const vector_t &stateDesired,
                                    const vector_t &inputDesired, const vector_t &desiredJointAcceleration);
    Task createSwingFootAccelerationTask(const vector_t &stateCurrent, const vector_t &inputCurrent,
                                         const vector_t &stateDesired, const vector_t &inputDesired);
    Task createContactForceMinimizationTask(const vector_t &inputDesired);

    void updateMeasuredState(const vector_t &stateCurrent, const vector_t &inputCurrent);
    void updateKinematicsAndDynamicsCurrent();
    void updateDesiredState(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateKinematicsAndDynamicsDesired(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateContactFlags(const size_t modeCurrent, const size_t modeDesired);

    /* Contact jacobians - stacked on top of each other */
    matrix_t Jcontact_;

    /* Contact jacobians time derivative - stacked on top of each other */
    matrix_t dJcontactdt_;

    /* Measured generalized positions */
    vector_t qMeasured_;

    /* Measured generalized velocities */
    vector_t vMeasured_;

    /* Measured rotation matrix from base to world*/
    matrix_t rMeasured_;

    /* Desired rotation matrix from base to world*/
    matrix_t rDesired_;

    /* Number of decision variables */
    size_t nDecisionVariables_;

    /* Number of generalized coordinates */
    size_t nGeneralizedCoordinates_;

    /* Measured pinocchio interface */
    ocs2::PinocchioInterface pinocchioInterfaceMeasured_;

    /* Foot names*/
    std::vector<std::string> footNames_;

    /* Friction cone matrix*/
    matrix_t muMatrix_;

    /* swing kp and kd*/
    scalar_t swingKp_;
    scalar_t swingKd_;

    /* base kp and kd */
    scalar_t baseKp_;
    scalar_t baseKd_;

    /* euler kp and kd*/
    scalar_t eulerKp_;
    scalar_t eulerKd_;

    /* torque limits */
    joint_coordinate_t torqueLimits_ = joint_coordinate_t::Constant(1.0);
    scalar_t torqueLimit_;

    bool stanceAsConstraint_ = true; // whether to enforce stance foot no-motion as hard constraint or soft constraint

    /* Desired contact flags */
    contact_flag_t contactFlags_;
    size_t nContacts_;

   private:
    void loadSettings(const std::string &configFile, const std::string &configPrefix);
    void generateFrictionConeMatrix(const scalar_t mu);

    std::unique_ptr<switched_model::ComModelBase<scalar_t>> comModelPtr_;
    std::unique_ptr<switched_model::KinematicsModelBase<scalar_t>> kinematicsPtr_;
};

}  // namespace switched_model

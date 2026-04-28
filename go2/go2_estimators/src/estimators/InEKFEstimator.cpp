/*! @file InEKFEstimator.cpp
 *  @brief Container for InEKF State Estimation Algorithm
 */


#include "go2_estimators/estimators/InEKFEstimator.h"

#include <ocs2_go2_mpc/Go2Interface.h>

namespace legged_software {
namespace go2_estimators {

/*!
 * Initialize the state estimator
 */

void InEKFEstimator::setup() 
{
  // Initialize pinocchio model
  // const std::string urdfString = "";

  // ocs2::PinocchioInterface pinInterface = ocs2::getPinocchioInterfaceFromUrdfFile(urdf);

  // if (urdfString.empty()) 
  // {
  //   pinocchio::urdf::buildModel(urdfFile, model_);
  // } 
  // else 
  // {
  //   pinocchio::urdf::buildModelFromXML(urdfString, model_);
  // }
  // data_ = pinocchio::Data(model_);

  // Instantiate NoiseParams
  go2_estimators::NoiseParams noise_params;
  noise_params.setGyroscopeNoise(Eigen::Vector3d(0.01, 0.01, 0.01));
  noise_params.setAccelerometerNoise(Eigen::Vector3d(0.1, 0.1, 0.1));
  noise_params.setGyroscopeBiasNoise(0.00001);
  noise_params.setAccelerometerBiasNoise(0.0001);
  noise_params.setContactNoise(0.001);
  inekf_.setNoiseParams(noise_params);

  // Set covariance matrix
  Eigen::MatrixXd P_init = Eigen::MatrixXd::Zero(15, 15);
  P_init.block<3, 3>(0, 0).diagonal().setConstant(pow(0.00001, 2));
  P_init.block<3, 3>(3, 3).diagonal().setConstant(pow(0.00001, 2));
  P_init.block<3, 3>(6, 6).diagonal().setConstant(pow(0.0000001, 2));
  P_init.block<3, 3>(9, 9).diagonal().setConstant(pow(0.001, 2));
  P_init.block<3, 3>(12, 12).diagonal().setConstant(pow(0.001, 2));

  // Instantiate RobotState
  go2_estimators::RobotState initial_state;
  initial_state.setRotation(Eigen::Matrix3d::Identity());
  initial_state.setVelocity(Eigen::Vector3d::Zero());
  initial_state.setPosition(Eigen::Vector3d(0.0, 0.0, 0.0));
  initial_state.setGyroscopeBias(Eigen::Vector3d(0.003, 0.0, -0.007));
  initial_state.setAccelerometerBias(Eigen::Vector3d(-0.08, 0.05, 0.3));
  initial_state.setP(P_init);
  inekf_.setState(initial_state);

  // Check if foot names exist in model
  // std::vector<std::string> footNames = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  // for (const auto &footName : footNames) 
  // {
  //   bool exists = model_.existBodyName(footName);
  //   if (!exists) 
  //   {
  //     std::vector<std::string> allFrames;
  //     for (const auto &frame : model_.frames) 
  //     {
  //       allFrames.push_back(frame.name);
  //     }
      
  //     RCLCPP_ERROR_STREAM(node_->get_logger(), "Cound not find body " << footName << " in model. All frames: ");
  //     for (const auto &frame : model_.frames) 
  //     {
  //       RCLCPP_ERROR_STREAM(node_->get_logger(), frame.name);
  //     }
  //   }

  //   legIndices_.push_back(model_.getBodyId(footName));
  // }  

  // RCLCPP_INFO_STREAM(node_->get_logger(), "Joint indices from Pinocchio model: ");
  // for (size_t i = 0; i < model_.frames.size(); ++i) 
  // {
  //     const auto &frame = model_.frames[i];
  //     RCLCPP_INFO_STREAM(node_->get_logger(), "Frame " << frame.name << ", index: " << i);
  // }  

  return;
}

/*
*   Constructor
*/

InEKFEstimator::InEKFEstimator(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  const std::string taskFile = node_->get_parameter("taskFile").as_string();
  const std::string urdfFile = node_->get_parameter("urdfFile").as_string();
  const std::string frameFile = node_->get_parameter("frameFile").as_string();
  std::string urdfString = go2::getUrdfString(urdfFile);
  legged_interface_ = go2::getGo2Interface(urdfString, taskFile, frameFile);
}


/*
 * Propagate IMU measurements
*/

void InEKFEstimator::propagate() 
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] propagate start");

    // Cast time interval to T
    double dt = this->_stateEstimatorData->parameters->estimator_dt;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   dt: " << dt);

    // Propagate IMU measurements
    Eigen::VectorXd imu_measurement(6);
    imu_measurement << lastAngularVelocity_, lastLinearAcceleration_;
    inekf_.Propagate(imu_measurement, dt);  
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] propagate end");
}

/*
*   Pass contact measurements to InEKF
*/

void InEKFEstimator::setContacts() 
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   setting contacts");

    // Create a vector to manage foot-contact pairs
    std::vector<std::pair<int, bool>> contacts_pairs = std::vector<std::pair<int, bool>>(4);

    // Check if each foot is in contact
    for (size_t i = 0; i < contacts_pairs.size(); i++) 
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Leg " << i << " contact: " << this->_stateEstimatorData->result.contactEstimate(i));
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Leg " << i << " contact flag: " << (this->_stateEstimatorData->result.contactEstimate(i) > 0));

        // Check contact estimate
        bool contactFlag = this->_stateEstimatorData->result.contactEstimate(i) > 0;

        // Save result
        contacts_pairs[i] = std::make_pair(i, contactFlag);
    }
    
    // Logging
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   contacts: ");
    // for (const auto &contact_pair : contacts_pairs)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "Leg " << contact_pair.first << ": " << (contact_pair.second ? "Contact" : "No Contact"));
    // }

    // Set contact measurements
    inekf_.setContacts(contacts_pairs);    
}

/*
 *  
 */

void InEKFEstimator::correctKinematics() 
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   updating kinematics");

    // Send joint data to pinocchio
    Eigen::VectorXd qPinocchio(12);
    for (size_t i = 0; i < 4; i++)
    {
      qPinocchio.segment(i * 3, 3) = this->_stateEstimatorData->limbData[i]->q.template cast<double>();
    }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   qPinocchio: " << qPinocchio.transpose());

    // Retrieve position estimate from InEKF
    // this->_stateEstimatorData->result.position = inekf_.getState().getPosition();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] position before updating pinocchio: " << this->_stateEstimatorData->result.position.transpose());

    // Compute forward kinematics with pinocchio
    // pinocchio::forwardKinematics(model_, data_, qPinocchio);

    // Retrieve position estimate from InEKF
    // this->_stateEstimatorData->result.position = inekf_.getState().getPosition();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] position after updating pinocchio: " << this->_stateEstimatorData->result.position.transpose());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   computing foot positions");

    // Update foot positions with pinocchio
    std::vector<Eigen::Vector3d> footPositions(4);
    for (size_t i = 0; i < legIndices_.size(); i++) 
    {
      footPositions[i] = legged_interface_->getKinematicModel().positionBaseToFootInBaseFrame(i, qPinocchio);
    //     footPositions[i] = pinocchio::updateFramePlacement(model_, data_, legIndices_[i]).translation();
    }    

    // Retrieve position estimate from InEKF
    // this->_stateEstimatorData->result.position = inekf_.getState().getPosition();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] position after updating foot positions: " << this->_stateEstimatorData->result.position.transpose());

    // Print foot positions
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   foot positions: ");
    // for (size_t i = 0; i < footPositions.size(); i++)
    // {
    //   RCLCPP_INFO_STREAM(node_->get_logger(), "Leg " << i << ": " << footPositions[i].transpose());
    // }   

    // Store predicted foot positions
    go2_estimators::vectorKinematics kinematics;
    kinematics.reserve(legIndices_.size());
    for (size_t i = 0; i < legIndices_.size(); i++) 
    {
        constexpr double foot_noise = 0.01;
        Eigen::Translation3d translation = Eigen::Translation3d(footPositions[i](0),footPositions[i](1), footPositions[i](2));
        Eigen::Transform<double, 3, Eigen::Affine> affine = Eigen::Transform<double, 3, Eigen::Affine>(translation);

        kinematics.emplace_back(i, affine.matrix(), Eigen::Matrix<double, 6, 6>::Identity() * pow(foot_noise, 2));
    }

    // Retrieve position estimate from InEKF
    // this->_stateEstimatorData->result.position = inekf_.getState().getPosition();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] position before correcting kinematics: " << this->_stateEstimatorData->result.position.transpose());

    // Correct InEKF estimate with estimated foot positions
    inekf_.CorrectKinematics(kinematics);     

    // Retrieve position estimate from InEKF
    // this->_stateEstimatorData->result.position = inekf_.getState().getPosition();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] position after correcting kinematics: " << this->_stateEstimatorData->result.position.transpose());
}

/*!
 * Run state estimator
 */

void InEKFEstimator::run() 
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator] run()");

    if (_b_first_visit)
    {
        // Angular velocity
        lastAngularVelocity_ = this->_stateEstimatorData->imuData->gyro;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]       lastAngularVelocity_: " << lastAngularVelocity_.transpose());

        // Linear Acceleration
        lastLinearAcceleration_ = this->_stateEstimatorData->imuData->accelerometer;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]       lastLinearAcceleration_: " << lastLinearAcceleration_.transpose());
    }

    if (rectifyOrientation_) 
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]   rectifying orientation");
        Eigen::Quaterniond q_ = this->_stateEstimatorData->result.orientation;
                              // (this->_stateEstimatorData->result.orientation[0],
                              //   this->_stateEstimatorData->result.orientation[1],
                              //   this->_stateEstimatorData->result.orientation[2],
                              //   this->_stateEstimatorData->result.orientation[3]);

        inekf_.getState().setRotation(q_.toRotationMatrix());
    }  

    // Propagate InEKF with IMU data
    this->propagate();

    // Set contacts
    this->setContacts();

    // Update state based on kinematics
    this->correctKinematics();

    // Angular velocity
    lastAngularVelocity_ = this->_stateEstimatorData->imuData->gyro;

    // Linear Acceleration
    lastLinearAcceleration_ = this->_stateEstimatorData->imuData->accelerometer;

    // Update state estimator data
    this->_stateEstimatorData->result.position = inekf_.getState().getPosition();
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]       position: " << this->_stateEstimatorData->result.position.transpose());

    this->_stateEstimatorData->result.vWorld = inekf_.getState().getVelocity();
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]       vWorld: " << this->_stateEstimatorData->result.vWorld.transpose());

    this->_stateEstimatorData->result.vBody = this->_stateEstimatorData->result.rBody * this->_stateEstimatorData->result.vWorld;
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[InEKFEstimator]       vBody: " << this->_stateEstimatorData->result.vBody.transpose());

  return;
}

} // namespace go2_estimators
} // namespace legged_software
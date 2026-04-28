/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for RobotState (thread-safe)
 *  @date   September 25, 2018
 **/

#include "go2_estimators/estimators/RobotState.h"
#include "go2_estimators/estimators/LieGroup.h"

namespace legged_software {
namespace go2_estimators {

using namespace std;

// Default constructor

RobotState::RobotState() : 
    X_(Eigen::MatrixXd::Identity(5,5)), Theta_(Eigen::MatrixXd::Zero(6,1)), P_(Eigen::MatrixXd::Identity(15,15)) {}

// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X) : 
    X_(X), Theta_(Eigen::MatrixXd::Zero(6,1)) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}

// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta) : 
    X_(X), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}

// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X), Theta_(Theta), P_(P) {}

// TODO: error checking to make sure dimensions are correct and supported


const Eigen::MatrixXd RobotState::getX() const { return X_; }
const Eigen::VectorXd RobotState::getTheta() const { return Theta_; }
const Eigen::MatrixXd RobotState::getP() const { return P_; }
const Eigen::Matrix3d RobotState::getRotation() const { return X_.block<3,3>(0,0); }
const Eigen::Vector3d RobotState::getVelocity() const { return X_.block<3,1>(0,3); }
const Eigen::Vector3d RobotState::getPosition() const { return X_.block<3,1>(0,4); }
const Eigen::Vector3d RobotState::getVector(int index) const { return X_.block<3,1>(0,index); }

const Eigen::Vector3d RobotState::getGyroscopeBias() const { return Theta_.head(3); }
const Eigen::Vector3d RobotState::getAccelerometerBias() const { return Theta_.tail(3); }

const int RobotState::dimX() const { return X_.cols(); }
const int RobotState::dimTheta() const {return Theta_.rows();}
const int RobotState::dimP() const { return P_.cols(); }

const StateType RobotState::getStateType() const { return state_type_; }

const Eigen::MatrixXd RobotState::getWorldX() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getX();
    } else {
        return this->Xinv();
    }
}
const Eigen::Matrix3d RobotState::getWorldRotation() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getRotation();
    } else {
        return this->getRotation().transpose();
    }
}
const Eigen::Vector3d RobotState::getWorldVelocity() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getVelocity();
    } else {
        return -this->getRotation().transpose()*this->getVelocity();
    }
}
const Eigen::Vector3d RobotState::getWorldPosition() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getPosition();
    } else {
        return -this->getRotation().transpose()*this->getPosition();
    }
}

const Eigen::MatrixXd RobotState::getBodyX() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getX();
    } else {
        return this->Xinv();
    }
}

const Eigen::Matrix3d RobotState::getBodyRotation() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getRotation();
    } else {
        return this->getRotation().transpose();
    }
}

const Eigen::Vector3d RobotState::getBodyVelocity() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getVelocity();
    } else {
        return -this->getRotation().transpose()*this->getVelocity();
    }
}

const Eigen::Vector3d RobotState::getBodyPosition() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getPosition();
    } else {
        return -this->getRotation().transpose()*this->getPosition();
    }
}

void RobotState::setX(const Eigen::MatrixXd& X) { X_ = X; }
void RobotState::setTheta(const Eigen::VectorXd& Theta) { Theta_ = Theta; }
void RobotState::setP(const Eigen::MatrixXd& P) { P_ = P; }
void RobotState::setRotation(const Eigen::Matrix3d& R) { X_.block<3,3>(0,0) = R; }
void RobotState::setVelocity(const Eigen::Vector3d& v) { X_.block<3,1>(0,3) = v; }
void RobotState::setPosition(const Eigen::Vector3d& p) { X_.block<3,1>(0,4) = p; }
void RobotState::setGyroscopeBias(const Eigen::Vector3d& bg) { Theta_.head(3) = bg; }
void RobotState::setAccelerometerBias(const Eigen::Vector3d& ba) { Theta_.tail(3) = ba; }

const Eigen::MatrixXd RobotState::Xinv() const {
    int dimX = this->dimX();
    Eigen::MatrixXd Xinv = Eigen::MatrixXd::Identity(dimX,dimX);
    Eigen::Matrix3d RT = X_.block<3,3>(0,0).transpose();
    Xinv.block<3,3>(0,0) = RT;
    for(int i=3; i<dimX; ++i) {
        Xinv.block<3,1>(0,i) = -RT*X_.block<3,1>(0,i);
    }
    return Xinv;
}

} // namespace go2_estimators
} // namespace legged_software
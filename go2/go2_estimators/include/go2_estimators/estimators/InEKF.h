/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include "go2_estimators/estimators/RobotState.h"
#include <go2_estimators/estimators/NoiseParams.h>

namespace legged_software {
namespace go2_estimators {

enum ErrorType {LeftInvariant, RightInvariant};


class Kinematics 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Kinematics(int id_in, Eigen::Matrix4d pose_in, Eigen::Matrix<double,6,6> covariance_in) : id(id_in), pose(pose_in), covariance(covariance_in) { }

        int id;
        Eigen::Matrix4d pose;
        Eigen::Matrix<double,6,6> covariance;

};

/** A map with an integer as key and a Eigen::Vector3d as value. */
typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > > mapIntVector3d;
typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > >::const_iterator mapIntVector3dIterator;

/** A vector of Kinematics. */
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> > vectorKinematics;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> >::const_iterator vectorKinematicsIterator;


class InEKF 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InEKF();
        InEKF(NoiseParams params);
        InEKF(RobotState state);
        InEKF(RobotState state, NoiseParams params);
        InEKF(RobotState state, NoiseParams params, ErrorType error_type);

        RobotState getState() const;
        NoiseParams getNoiseParams() const;
        // std::map<int,int> getEstimatedLandmarks();
        std::map<int,bool> getContacts() const;
        std::map<int,int> getEstimatedContactPositions() const;
        void setState(RobotState state);
        void setNoiseParams(NoiseParams params);
        void setContacts(std::vector<std::pair<int,bool> > contacts);

        /**
         * Resets the filter
         * Initializes state matrix to identity, removes all augmented states, and assigns default noise parameters.
         */
        void clear();        

        void Propagate(const Eigen::Matrix<double,6,1>& imu, double dt);
        void CorrectKinematics(const vectorKinematics& measured_kinematics);

        Eigen::MatrixXd StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt);
        Eigen::MatrixXd DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt);

        void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N);
        void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N);

    private:
        ErrorType error_type_ = ErrorType::LeftInvariant;
        bool estimate_bias_ = true;  
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
};

} // namespace go2_estimators
} // namespace legged_software
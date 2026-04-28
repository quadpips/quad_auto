/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#include "go2_estimators/estimators/InEKF.h"

namespace legged_software {
namespace go2_estimators {

using namespace std;

long int factorial(int n);


Eigen::Matrix3d skew(const Eigen::Vector3d& v);


Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, int n);


Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);


Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v);


Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);


void removeRowAndColumn(Eigen::MatrixXd& M, int index);

// ------------ InEKF -------------
// Default constructor

InEKF::InEKF() : g_((Eigen::Vector3d(3) << 0,0,-9.81).finished()) {}

// Constructor with noise params

InEKF::InEKF(NoiseParams params) : g_((Eigen::Vector3d(3) << 0,0,-9.81).finished()), noise_params_(params) {}

// Constructor with initial state

InEKF::InEKF(RobotState state) : g_((Eigen::Vector3d(3) << 0,0,-9.81).finished()), state_(state) {}

// Constructor with initial state and noise params

InEKF::InEKF(RobotState state, NoiseParams params) : g_((Eigen::Vector3d(3) << 0,0,-9.81).finished()), state_(state), noise_params_(params) {}

// Constructor with initial state, noise params, and error type

InEKF::InEKF(RobotState state, NoiseParams params, ErrorType error_type) : 
    g_((Eigen::Vector3d(3) << 0,0,-9.81).finished()), 
    state_(state), 
    noise_params_(params), 
    error_type_(error_type) {}

// Clear all data in the filter

void InEKF::clear() 
{
    state_ = RobotState();
    noise_params_ = NoiseParams();
    // prior_landmarks_.clear();
    // estimated_landmarks_.clear();
    contacts_.clear();
    estimated_contact_positions_.clear();
}

// Return robot's current state

RobotState InEKF::getState() const 
{ 
    return state_; 
}

// Sets the robot's current state

void InEKF::setState(RobotState state) 
{ 
    state_ = state;
}

// Return noise params

NoiseParams InEKF::getNoiseParams() const 
{ 
    return noise_params_; 
}

// Sets the filter's noise parameters

void InEKF::setNoiseParams(NoiseParams params) 
{ 
    noise_params_ = params; 
}

// Return filter's estimated contact positions

map<int,int> InEKF::getEstimatedContactPositions() const 
{ 
    return estimated_contact_positions_; 
}

// Set the filter's contact state

void InEKF::setContacts(vector<pair<int,bool> > contacts) 
{
    // Insert new measured contact states
    for (vector<pair<int,bool> >::iterator it=contacts.begin(); it!=contacts.end(); ++it) {
        pair<map<int,bool>::iterator,bool> ret = contacts_.insert(*it);
        // If contact is already in the map, replace with new value
        if (ret.second==false) {
            ret.first->second = it->second;
        }
    }
    return;
}

// Return the filter's contact state

std::map<int,bool> InEKF::getContacts() const
{
    return contacts_; 
}

// Compute Analytical state transition matrix

Eigen::MatrixXd InEKF::StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt)
{
    Eigen::Vector3d phi = w*dt;
    Eigen::Vector3d neg_phi = -phi;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi,1); // TODO: These are also needed for the mean propagation, we should not compute twice
    Eigen::Matrix3d G2 = Gamma_SO3(phi,2);
    Eigen::Matrix3d G0t = G0.transpose();
    Eigen::Matrix3d G1t = G1.transpose();
    Eigen::Matrix3d G2t = G2.transpose();
    Eigen::Matrix3d G3t = Gamma_SO3(neg_phi,3);
    
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(dimP,dimP);

    // Compute the complicated bias terms (derived for the left invariant case)
    Eigen::Matrix3d ax = skew(a);
    Eigen::Matrix3d wx = skew(w);
    Eigen::Matrix3d wx2 = wx*wx;
    double dt2 = dt*dt;
    double dt3 = dt2*dt;
    double theta = w.norm();
    double theta2 = theta*theta;
    double theta3 = theta2*theta;
    double theta4 = theta3*theta;
    double theta5 = theta4*theta;
    double theta6 = theta5*theta;
    double theta7 = theta6*theta;
    double thetadt = theta*dt;
    double thetadt2 = thetadt*thetadt;
    double thetadt3 = thetadt2*thetadt;
    double sinthetadt = sin(thetadt);
    double costhetadt = cos(thetadt);
    double sin2thetadt = sin(2*thetadt);
    double cos2thetadt = cos(2*thetadt);
    double thetadtcosthetadt = thetadt*costhetadt;
    double thetadtsinthetadt = thetadt*sinthetadt;

    Eigen::Matrix3d Phi25L = G0t*(ax*G2t*dt2 
        + ((sinthetadt-thetadtcosthetadt)/(theta3))*(wx*ax)
        - ((cos2thetadt-4*costhetadt+3)/(4*theta4))*(wx*ax*wx)
        + ((4*sinthetadt+sin2thetadt-4*thetadtcosthetadt-2*thetadt)/(4*theta5))*(wx*ax*wx2)
        + ((thetadt2-2*thetadtsinthetadt-2*costhetadt+2)/(2*theta4))*(wx2*ax)
        - ((6*thetadt-8*sinthetadt+sin2thetadt)/(4*theta5))*(wx2*ax*wx)
        + ((2*thetadt2-4*thetadtsinthetadt-cos2thetadt+1)/(4*theta6))*(wx2*ax*wx2) );

    Eigen::Matrix3d Phi35L = G0t*(ax*G3t*dt3
        - ((thetadtsinthetadt+2*costhetadt-2)/(theta4))*(wx*ax)
        - ((6*thetadt-8*sinthetadt+sin2thetadt)/(8*theta5))*(wx*ax*wx)
        - ((2*thetadt2+8*thetadtsinthetadt+16*costhetadt+cos2thetadt-17)/(8*theta6))*(wx*ax*wx2)
        + ((thetadt3+6*thetadt-12*sinthetadt+6*thetadtcosthetadt)/(6*theta5))*(wx2*ax)
        - ((6*thetadt2+16*costhetadt-cos2thetadt-15)/(8*theta6))*(wx2*ax*wx)
        + ((4*thetadt3+6*thetadt-24*sinthetadt-3*sin2thetadt+24*thetadtcosthetadt)/(24*theta7))*(wx2*ax*wx2) );

    
    // TODO: Get better approximation using taylor series when theta < tol
    const double tol =  1e-6;
    if (theta < tol) 
    {
        Phi25L = (1/2)*ax*dt2;
        Phi35L = (1/6)*ax*dt3;
    }

    // Fill out analytical state transition matrices
    if  ((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::LeftInvariant) || 
         (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::RightInvariant)) {
        // Compute left-invariant state transisition matrix
        Phi.template block<3,3>(0,0) = G0t; // Phi_11
        Eigen::Vector3d G1a = G1*a;
        Eigen::Vector3d G2a = G2*a;
        Phi.template block<3,3>(3,0) = -G0t*skew(G1a)*dt; // Phi_21
        Phi.template block<3,3>(6,0) = -G0t*skew(G2a)*dt2; // Phi_31
        Phi.template block<3,3>(3,3) = G0t; // Phi_22
        Phi.template block<3,3>(6,3) = G0t*dt; // Phi_32
        Phi.template block<3,3>(6,6) = G0t; // Phi_33
        for (int i=5; i<dimX; ++i) {
            Phi.template block<3,3>((i-2)*3,(i-2)*3) = G0t; // Phi_(3+i)(3+i)
        }
        Phi.template block<3,3>(0,dimP-dimTheta) = -G1t*dt; // Phi_15
        Phi.template block<3,3>(3,dimP-dimTheta) = Phi25L; // Phi_25
        Phi.template block<3,3>(6,dimP-dimTheta) = Phi35L; // Phi_35
        Phi.template block<3,3>(3,dimP-dimTheta+3) = -G1t*dt; // Phi_26
        Phi.template block<3,3>(6,dimP-dimTheta+3) = -G0t*G2*dt2; // Phi_36
    } else {
        // Compute right-invariant state transition matrix (Assumes unpropagated state)
        Eigen::Matrix3d gx = skew(g_);
        Eigen::Matrix3d R = state_.getRotation();
        Eigen::Vector3d v = state_.getVelocity();
        Eigen::Vector3d p = state_.getPosition();
        Eigen::Matrix3d RG0 = R*G0;
        Eigen::Matrix3d RG1dt = R*G1*dt;
        Eigen::Matrix3d RG2dt2 = R*G2*dt2;
        Phi.template block<3,3>(3,0) = gx*dt; // Phi_21
        Phi.template block<3,3>(6,0) = 0.5*gx*dt2; // Phi_31
        Phi.template block<3,3>(6,3) = Eigen::Matrix3d::Identity()*dt; // Phi_32
        Phi.template block<3,3>(0,dimP-dimTheta) = -RG1dt; // Phi_15
        Eigen::Vector3d prod1 = v + RG1dt*a + g_*dt;
        Eigen::Vector3d prod2 = p + v*dt + RG2dt2*a + 0.5*g_*dt2;
        Phi.template block<3,3>(3,dimP-dimTheta) = -skew(prod1)*RG1dt + RG0*Phi25L; // Phi_25
        Phi.template block<3,3>(6,dimP-dimTheta) = -skew(prod2)*RG1dt + RG0*Phi35L; // Phi_35
        for (int i=5; i<dimX; ++i) {
            Phi.template block<3,3>((i-2)*3,dimP-dimTheta) = -skew(state_.getVector(i))*RG1dt; // Phi_(3+i)5
        }
        Phi.template block<3,3>(3,dimP-dimTheta+3) = -RG1dt; // Phi_26
        Phi.template block<3,3>(6,dimP-dimTheta+3) = -RG2dt2; // Phi_36
    }
    return Phi;
}

// Compute Discrete noise matrix

Eigen::MatrixXd InEKF::DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt)
{
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();    
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(dimP,dimP);

    // Compute G using Adjoint of Xk if needed, otherwise identity (Assumes unpropagated state)
    if  ((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant) || 
         (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
        G.template block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.getWorldX()); 
    }

    // Continuous noise covariance 
    Eigen::MatrixXd Qc = Eigen::MatrixXd::Zero(dimP,dimP); // Landmark noise terms will remain zero
    Qc.template block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
    Qc.template block<3,3>(3,3) = noise_params_.getAccelerometerCov();
    for(map<int,int>::iterator it=estimated_contact_positions_.begin(); it!=estimated_contact_positions_.end(); ++it) {
        Qc.template block<3,3>(3+3*(it->second-3),3+3*(it->second-3)) = noise_params_.getContactCov(); // Contact noise terms
    } // TODO: Use kinematic orientation to map noise from contact frame to body frame (not needed if noise is isotropic)
    Qc.template block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qc.template block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();

    // Noise Covariance Discretization
    Eigen::MatrixXd PhiG = Phi * G;
    Eigen::MatrixXd Qd = PhiG * Qc * PhiG.transpose() * dt; // Approximated discretized noise matrix (TODO: compute analytical)
    return Qd;
}


// InEKF Propagation - Inertial Data

void InEKF::Propagate(const Eigen::Matrix<double,6,1>& imu, double dt)
{

    // Bias corrected IMU measurements
    Eigen::Vector3d w = imu.head(3)  - state_.getGyroscopeBias();    // Angular Velocity
    Eigen::Vector3d a = imu.tail(3) - state_.getAccelerometerBias(); // Linear Acceleration
    
    // Get current state estimate and dimensions
    Eigen::MatrixXd X = state_.getX();
    Eigen::MatrixXd Xinv = state_.Xinv();
    Eigen::MatrixXd P = state_.getP();
    int dimX = state_.dimX();
    int dimP = state_.dimP();
    int dimTheta = state_.dimTheta();

    //  ------------ Propagate Covariance --------------- //
    Eigen::MatrixXd Phi = this->StateTransitionMatrix(w,a,dt);
    Eigen::MatrixXd Qd = this->DiscreteNoiseMatrix(Phi, dt);
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qd;

    // If we don't want to estimate bias, remove correlation
    if (!estimate_bias_) {
        P_pred.template block(0,dimP-dimTheta,dimP-dimTheta,dimTheta) = Eigen::MatrixXd::Zero(dimP-dimTheta,dimTheta);
        P_pred.template block(dimP-dimTheta,0,dimTheta,dimP-dimTheta) = Eigen::MatrixXd::Zero(dimTheta,dimP-dimTheta);
        P_pred.template block(dimP-dimTheta,dimP-dimTheta,dimTheta,dimTheta) = Eigen::MatrixXd::Identity(dimTheta,dimTheta);
    }    

    //  ------------ Propagate Mean --------------- // 
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getVelocity();
    Eigen::Vector3d p = state_.getPosition();

    Eigen::Vector3d phi = w*dt;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi,1);
    Eigen::Matrix3d G2 = Gamma_SO3(phi,2);

    Eigen::MatrixXd X_pred = X;
    if (state_.getStateType() == StateType::WorldCentric) {
        // Propagate world-centric state estimate
        X_pred.template block<3,3>(0,0) = R * G0;
        X_pred.template block<3,1>(0,3) = v + (R*G1*a + g_)*dt;
        X_pred.template block<3,1>(0,4) = p + v*dt + (R*G2*a + 0.5*g_)*dt*dt;
    } else {
        // Propagate body-centric state estimate
        Eigen::MatrixXd X_pred = X;
        Eigen::Matrix3d G0t = G0.transpose();
        X_pred.template block<3,3>(0,0) = G0t*R;
        X_pred.template block<3,1>(0,3) = G0t*(v - (G1*a + R*g_)*dt);
        X_pred.template block<3,1>(0,4) = G0t*(p + v*dt - (G2*a + 0.5*R*g_)*dt*dt);
        for (int i=5; i<dimX; ++i) {
            X_pred.template block<3,1>(0,i) = G0t*X.template block<3,1>(0,i);
        }
    } 

    //  ------------ Update State --------------- // 
    state_.setX(X_pred);
    state_.setP(P_pred);      

    return;
}

// // Correct State: Right-Invariant Observation
// 
// void InEKF::Correct(const Observation<T>& obs) {
//     // Compute Kalman Gain
//     Eigen::MatrixXd P = state_.getP();
//     Eigen::MatrixXd PHT = P * obs.H.transpose();
//     Eigen::MatrixXd S = obs.H * PHT + obs.N;
//     Eigen::MatrixXd K = PHT * S.inverse();

//     // Copy X along the diagonals if more than one measurement
//     Eigen::MatrixXd BigX;
//     state_.copyDiagX(obs.Y.rows()/state_.dimX(), BigX);
   
//     // Compute correction terms
//     Eigen::MatrixXd Z = BigX*obs.Y - obs.b;
//     Eigen::VectorXd delta = K*obs.PI*Z;
//     Eigen::VectorXd dX_vec = delta.template segment(0,delta.rows()-state_.dimTheta());
//     Eigen::MatrixXd dX = Exp_SEK3(dX_vec);
//     Eigen::VectorXd dTheta = delta.template segment(delta.rows()-state_.dimTheta(), state_.dimTheta());

//     // Update state
//     Eigen::MatrixXd X_new = dX*state_.getX(); // Right-Invariant Update
//     Eigen::VectorXd Theta_new = state_.getTheta() + dTheta;
//     state_.setX(X_new); 
//     state_.setTheta(Theta_new);

//     // Update Covariance
//     Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_.dimP(),state_.dimP()) - K*obs.H;
//     Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*obs.N*K.transpose(); // Joseph update form

//     state_.setP(P_new); 
// }   

// // Create Observation from vector of landmark measurements
// 
// void InEKF::CorrectLandmarks(const vectorLandmarks<T>& measured_landmarks) {
// #if INEKF_USE_MUTEX
//     lock_guard<mutex> mlock(estimated_landmarks_mutex_);
// #endif
//     Eigen::VectorXd Y;
//     Eigen::VectorXd b;
//     Eigen::MatrixXd H;
//     Eigen::MatrixXd N;
//     Eigen::MatrixXd PI;

//     Eigen::Matrix3d R = state_.getRotation();
//     vectorLandmarks<T> new_landmarks;
//     vector<int> used_landmark_ids;
    
//     for (vectorLandmarksIterator<T> it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
//         // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
//         if (find(used_landmark_ids.begin(), used_landmark_ids.end(), it->id) != used_landmark_ids.end()) { 
//             cout << "Duplicate landmark ID detected! Skipping measurement.\n";
//             continue; 
//         } else { used_landmark_ids.push_back(it->id); }

//         // See if we can find id in prior_landmarks or estimated_landmarks
//         mapIntVector3dIterator<T> it_prior = prior_landmarks_.find(it->id);
//         map<int,int>::iterator it_estimated = estimated_landmarks_.find(it->id);
//         if (it_prior!=prior_landmarks_.end()) {
//             // Found in prior landmark set
//             int dimX = state_.dimX();
//             int dimP = state_.dimP();
//             int startIndex;

//             // Fill out Y
//             startIndex = Y.rows();
//             Y.conservativeResize(startIndex+dimX, Eigen::NoChange);
//             Y.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
//             Y.segment(startIndex,3) = it->position; // p_bl
//             Y(startIndex+4) = 1; 

//             // Fill out b
//             startIndex = b.rows();
//             b.conservativeResize(startIndex+dimX, Eigen::NoChange);
//             b.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
//             b.segment(startIndex,3) = it_prior->second; // p_wl
//             b(startIndex+4) = 1;       

//             // Fill out H
//             startIndex = H.rows();
//             H.conservativeResize(startIndex+3, dimP);
//             H.template block(startIndex,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
//             H.template block(startIndex,0,3,3) = skew(it_prior->second); // skew(p_wl)
//             H.template block(startIndex,6,3,3) = -Eigen::Matrix3d::Identity(); // -I

//             // Fill out N
//             startIndex = N.rows();
//             N.conservativeResize(startIndex+3, startIndex+3);
//             N.template block(startIndex,0,3,startIndex) = Eigen::MatrixXd::Zero(3,startIndex);
//             N.template block(0,startIndex,startIndex,3) = Eigen::MatrixXd::Zero(startIndex,3);
//             N.template block(startIndex,startIndex,3,3) = R * noise_params_.getLandmarkCov() * R.transpose();

//             // Fill out PI      
//             startIndex = PI.rows();
//             int startIndex2 = PI.cols();
//             PI.conservativeResize(startIndex+3, startIndex2+dimX);
//             PI.template block(startIndex,0,3,startIndex2) = Eigen::MatrixXd::Zero(3,startIndex2);
//             PI.template block(0,startIndex2,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
//             PI.template block(startIndex,startIndex2,3,dimX) = Eigen::MatrixXd::Zero(3,dimX);
//             PI.template block(startIndex,startIndex2,3,3) = Eigen::Matrix3d::Identity();

//         } else if (it_estimated!=estimated_landmarks_.end()) {;
//             // Found in estimated landmark set
//             int dimX = state_.dimX();
//             int dimP = state_.dimP();
//             int startIndex;

//             // Fill out Y
//             startIndex = Y.rows();
//             Y.conservativeResize(startIndex+dimX, Eigen::NoChange);
//             Y.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
//             Y.segment(startIndex,3) = it->position; // p_bl
//             Y(startIndex+4) = 1; 
//             Y(startIndex+it_estimated->second) = -1;       

//             // Fill out b
//             startIndex = b.rows();
//             b.conservativeResize(startIndex+dimX, Eigen::NoChange);
//             b.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
//             b(startIndex+4) = 1;       
//             b(startIndex+it_estimated->second) = -1;       

//             // Fill out H
//             startIndex = H.rows();
//             H.conservativeResize(startIndex+3, dimP);
//             H.template block(startIndex,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
//             H.template block(startIndex,6,3,3) = -Eigen::Matrix3d::Identity(); // -I
//             H.template block(startIndex,3*it_estimated->second-6,3,3) = Eigen::Matrix3d::Identity(); // I

//             // Fill out N
//             startIndex = N.rows();
//             N.conservativeResize(startIndex+3, startIndex+3);
//             N.template block(startIndex,0,3,startIndex) = Eigen::MatrixXd::Zero(3,startIndex);
//             N.template block(0,startIndex,startIndex,3) = Eigen::MatrixXd::Zero(startIndex,3);
//             N.template block(startIndex,startIndex,3,3) = R * noise_params_.getLandmarkCov() * R.transpose();

//             // Fill out PI      
//             startIndex = PI.rows();
//             int startIndex2 = PI.cols();
//             PI.conservativeResize(startIndex+3, startIndex2+dimX);
//             PI.template block(startIndex,0,3,startIndex2) = Eigen::MatrixXd::Zero(3,startIndex2);
//             PI.template block(0,startIndex2,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
//             PI.template block(startIndex,startIndex2,3,dimX) = Eigen::MatrixXd::Zero(3,dimX);
//             PI.template block(startIndex,startIndex2,3,3) = Eigen::Matrix3d::Identity();


//         } else {
//             // First time landmark as been detected (add to list for later state augmentation)
//             new_landmarks.push_back(*it);
//         }
//     }

//     // Correct state using stacked observation
//     Observation obs(Y,b,H,N,PI);
//     if (!obs.empty()) {
//         this->Correct(obs);
//     }

//     // Augment state with newly detected landmarks
//     if (new_landmarks.size() > 0) {
//         Eigen::MatrixXd X_aug = state_.getX(); 
//         Eigen::MatrixXd P_aug = state_.getP();
//         Eigen::Vector3d p = state_.getPosition();
//         for (vectorLandmarksIterator<T> it=new_landmarks.begin(); it!=new_landmarks.end(); ++it) {
//             // Initialize new landmark mean
//             int startIndex = X_aug.rows();
//             X_aug.conservativeResize(startIndex+1, startIndex+1);
//             X_aug.template block(startIndex,0,1,startIndex) = Eigen::MatrixXd::Zero(1,startIndex);
//             X_aug.template block(0,startIndex,startIndex,1) = Eigen::MatrixXd::Zero(startIndex,1);
//             X_aug(startIndex, startIndex) = 1;
//             X_aug.template block(0,startIndex,3,1) = p + R*it->position;

//             // Initialize new landmark covariance - TODO:speed up
//             Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_.dimP()+3,state_.dimP()); 
//             F.template block(0,0,state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()); // for old X
//             F.template block(state_.dimP()-state_.dimTheta(),6,3,3) = Eigen::Matrix3d::Identity(); // for new landmark
//             F.template block(state_.dimP()-state_.dimTheta()+3,state_.dimP()-state_.dimTheta(),state_.dimTheta(),state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimTheta(),state_.dimTheta()); // for theta
//             Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(),3);
//             G.template block(G.rows()-state_.dimTheta()-3,0,3,3) = R;
//             P_aug = (F*P_aug*F.transpose() + G*noise_params_.getLandmarkCov()*G.transpose()).eval();

//             // Update state and covariance
//             state_.setX(X_aug);
//             state_.setP(P_aug);

//             // Add to list of estimated landmarks
//             estimated_landmarks_.insert(pair<int,int> (it->id, startIndex));
//         }
//     }
//     return;    
// }


// Correct State: Right-Invariant Observation

void InEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N) 
{
    // Get current state estimate
    Eigen::MatrixXd X = state_.getX();
    Eigen::VectorXd Theta = state_.getTheta();
    Eigen::MatrixXd P = state_.getP();
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();

    // Remove bias
    Theta = Eigen::VectorXd::Zero(6);
    P.template block<6,6>(dimP-dimTheta,dimP-dimTheta) = 0.0001*Eigen::MatrixXd::Identity(6,6);
    P.template block(0,dimP-dimTheta,dimP-dimTheta,dimTheta) = Eigen::MatrixXd::Zero(dimP-dimTheta,dimTheta);
    P.template block(dimP-dimTheta,0,dimTheta,dimP-dimTheta) = Eigen::MatrixXd::Zero(dimTheta,dimP-dimTheta);
    // std::cout << "P:\n" << P << std::endl;
    // std::cout << state_ << std::endl;

    // Map from left invariant to right invariant error temporarily
    if (error_type_==ErrorType::LeftInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP,dimP);
        Adj.template block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); 
        P = (Adj * P * Adj.transpose()).eval(); 
    }

    // Compute Kalman Gain
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute state correction vector
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = dX*X; // Right-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state  
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP,dimP) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

    // Map from right invariant back to left invariant error
    if (error_type_==ErrorType::LeftInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP,dimP);
        AdjInv.template block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.Xinv()); 
        P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
    }

    // Set new covariance
    state_.setP(P_new); 
}   


// Correct State: Left-Invariant Observation

void InEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N) 
{
    // Get current state estimate
    Eigen::MatrixXd X = state_.getX();
    Eigen::VectorXd Theta = state_.getTheta();
    Eigen::MatrixXd P = state_.getP();
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();

    // Map from right invariant to left invariant error temporarily
    if (error_type_==ErrorType::RightInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP,dimP);
        AdjInv.template block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.Xinv()); 
        P = (AdjInv * P * AdjInv.transpose()).eval();
    }

    // Compute Kalman Gain
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute state correction vector
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = X*dX; // Left-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP,dimP) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

    // Map from left invariant back to right invariant error
    if (error_type_==ErrorType::RightInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP,dimP);
        Adj.template block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X_new); 
        P_new = (Adj * P_new * Adj.transpose()).eval(); 
    }

    // Set new covariance
    state_.setP(P_new); 
}   

// Correct state using kinematics measured between imu and contact point

void InEKF::CorrectKinematics(const vectorKinematics& measured_kinematics)
{
    Eigen::VectorXd Z, Y, b;
    Eigen::MatrixXd H, N, PI;

    vector<pair<int,int> > remove_contacts;
    vectorKinematics new_contacts;
    vector<int> used_contact_ids;

   for (vectorKinematicsIterator it=measured_kinematics.begin(); it!=measured_kinematics.end(); ++it) 
   {
        // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
        if (find(used_contact_ids.begin(), used_contact_ids.end(), it->id) != used_contact_ids.end()) { 
            cout << "Duplicate contact ID detected! Skipping measurement.\n";
            continue; 
        } else { used_contact_ids.push_back(it->id); }

        // Find contact indicator for the kinematics measurement
        map<int,bool>::iterator it_contact = contacts_.find(it->id);
        if (it_contact == contacts_.end()) { continue; } // Skip if contact state is unknown
        bool contact_indicated = it_contact->second;

        // See if we can find id estimated_contact_positions
        map<int,int>::iterator it_estimated = estimated_contact_positions_.find(it->id);
        bool found = it_estimated!=estimated_contact_positions_.end();

        // If contact is not indicated and id is found in estimated_contacts_, then remove state
        if (!contact_indicated && found) {
            remove_contacts.push_back(*it_estimated); // Add id to remove list
        //  If contact is indicated and id is not found i n estimated_contacts_, then augment state
        } else if (contact_indicated && !found) {
            new_contacts.push_back(*it); // Add to augment list

        // If contact is indicated and id is found in estimated_contacts_, then correct using kinematics
        } else if (contact_indicated && found) {
            int dimX = state_.dimX();
            int dimTheta = state_.dimTheta();
            int dimP = state_.dimP();
            int startIndex;

            // Fill out H
            startIndex = H.rows();
            H.conservativeResize(startIndex+3, dimP);
            H.template block(startIndex,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
            if (state_.getStateType() == StateType::WorldCentric) {
                H.template block(startIndex,6,3,3) = -Eigen::Matrix3d::Identity(); // -I
                H.template block(startIndex,3*it_estimated->second-dimTheta,3,3) = Eigen::Matrix3d::Identity(); // I
            } else {
                H.template block(startIndex,6,3,3) = Eigen::Matrix3d::Identity(); // I
                H.template block(startIndex,3*it_estimated->second-dimTheta,3,3) = -Eigen::Matrix3d::Identity(); // -I
            }
            
            // Fill out N
            startIndex = N.rows();
            N.conservativeResize(startIndex+3, startIndex+3);
            N.template block(startIndex,0,3,startIndex) = Eigen::MatrixXd::Zero(3,startIndex);
            N.template block(0,startIndex,startIndex,3) = Eigen::MatrixXd::Zero(startIndex,3);
            N.template block(startIndex,startIndex,3,3) = state_.getWorldRotation() * it->covariance.template block<3,3>(3,3) * state_.getWorldRotation().transpose();
    
            // Fill out Z
            startIndex = Z.rows();
            Z.conservativeResize(startIndex+3, Eigen::NoChange);
            Eigen::Matrix3d R = state_.getRotation();
            Eigen::Vector3d p = state_.getPosition();
            Eigen::Vector3d d = state_.getVector(it_estimated->second);  
            if (state_.getStateType() == StateType::WorldCentric) {
                Z.template segment(startIndex,3) = R*it->pose.template block<3,1>(0,3) - (d - p); 
            } else {
                Z.template segment(startIndex,3) = R.transpose()*(it->pose.template block<3,1>(0,3) - (p - d)); 
            }

        //  If contact is not indicated and id is found in estimated_contacts_, then skip
        } else {
            continue;
        }
    }

    // Correct state using stacked observation
    if (Z.rows()>0) {
        if (state_.getStateType() == StateType::WorldCentric) {
            this->CorrectRightInvariant(Z,H,N);
            // this->CorrectRightInvariant(obs);
        } else {
            // this->CorrectLeftInvariant(obs);
            this->CorrectLeftInvariant(Z,H,N);
        }
    }

    // Remove contacts from state
    if (remove_contacts.size() > 0) {
        Eigen::MatrixXd X_rem = state_.getX(); 
        Eigen::MatrixXd P_rem = state_.getP();
        for (vector<pair<int,int> >::iterator it=remove_contacts.begin(); it!=remove_contacts.end(); ++it) {
            // Remove row and column from X
            removeRowAndColumn(X_rem, it->second);
            // Remove 3 rows and columns from P
            int startIndex = 3 + 3*(it->second-3);
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            // Update all indices for estimated_landmarks and estimated_contact_positions
            // for (map<int,int>::iterator it2=estimated_landmarks_.begin(); it2!=estimated_landmarks_.end(); ++it2) {
            //     if (it2->second > it->second) 
            //         it2->second -= 1;
            // }
            for (map<int,int>::iterator it2=estimated_contact_positions_.begin(); it2!=estimated_contact_positions_.end(); ++it2) {
                if (it2->second > it->second) 
                    it2->second -= 1;
            }
            // We also need to update the indices of remove_contacts in the case where multiple contacts are being removed at once
            for (vector<pair<int,int> >::iterator it2=it; it2!=remove_contacts.end(); ++it2) {
                if (it2->second > it->second) 
                    it2->second -= 1;
            }
            // Remove from list of estimated contact positions 
            estimated_contact_positions_.erase(it->first);
        }
        // Update state and covariance
        state_.setX(X_rem);
        state_.setP(P_rem);
    }


    // Augment state with newly detected contacts
    if (new_contacts.size() > 0) {
        Eigen::MatrixXd X_aug = state_.getX(); 
        Eigen::MatrixXd P_aug = state_.getP();
        for (vectorKinematicsIterator it=new_contacts.begin(); it!=new_contacts.end(); ++it) {
            // Initialize new landmark mean
            int startIndex = X_aug.rows();
            X_aug.conservativeResize(startIndex+1, startIndex+1);
            X_aug.template block(startIndex,0,1,startIndex) = Eigen::MatrixXd::Zero(1,startIndex);
            X_aug.template block(0,startIndex,startIndex,1) = Eigen::MatrixXd::Zero(startIndex,1);
            X_aug(startIndex, startIndex) = 1;

            Eigen::Vector3d foot_position = it->pose.template block<3,1>(0,3);
            Eigen::Vector3d neg_foot_position = -foot_position;
            if (state_.getStateType() == StateType::WorldCentric) {
                X_aug.template block(0,startIndex,3,1) = state_.getPosition() + state_.getRotation()*foot_position;
            } else {
                X_aug.template block(0,startIndex,3,1) = state_.getPosition() - foot_position;
            }

            // Initialize new landmark covariance - TODO:speed up
            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_.dimP()+3,state_.dimP()); 
            F.template block(0,0,state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()); // for old X
            F.template block(state_.dimP()-state_.dimTheta()+3,state_.dimP()-state_.dimTheta(),state_.dimTheta(),state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimTheta(),state_.dimTheta()); // for theta
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(),3);
            // Blocks for new contact
            if ((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant) || 
                (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
                F.template block(state_.dimP()-state_.dimTheta(),6,3,3) = Eigen::Matrix3d::Identity(); 
                G.template block(G.rows()-state_.dimTheta()-3,0,3,3) = state_.getWorldRotation();
            } else {
                F.template block(state_.dimP()-state_.dimTheta(),6,3,3) = Eigen::Matrix3d::Identity(); 
                F.template block(state_.dimP()-state_.dimTheta(),0,3,3) = skew(neg_foot_position);
                G.template block(G.rows()-state_.dimTheta()-3,0,3,3) = Eigen::Matrix3d::Identity();
            }
            P_aug = (F*P_aug*F.transpose() + G*it->covariance.template block<3,3>(3,3)*G.transpose()).eval(); 

            // Update state and covariance
            state_.setX(X_aug); // TODO: move outside of loop (need to make loop independent of state_)
            state_.setP(P_aug);

            // Add to list of estimated contact positions
            estimated_contact_positions_.insert(pair<int,int> (it->id, startIndex));
        }
    }
}

const double TOLERANCE = 1e-10;

long int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}


Eigen::Matrix3d skew(const Eigen::Vector3d& v) 
{
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
        return M;
}


Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, int m) {
    // Computes mth integral of the exponential map: \Gamma_m = \sum_{n=0}^{\infty} \dfrac{1}{(n+m)!} (w^\wedge)^n
    assert(m>=0);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta < TOLERANCE) 
    {
        return (1.0/factorial(m))*I; // TODO: There is a better small value approximation for exp() given in Trawny p.19
    } 
    Eigen::Matrix3d A = skew(w);
    double theta2 =  theta*theta;

    // Closed form solution for the first 3 cases
    switch (m) 
    {
        case 0: // Exp map of SO(3)
            return I + (sin(theta)/theta)*A + ((1-cos(theta))/theta2)*A*A;
        
        case 1: // Left Jacobian of SO(3)
            // eye(3) - A*(1/theta^2) * (R - eye(3) - A);
            // eye(3) + (1-cos(theta))/theta^2 * A + (theta-sin(theta))/theta^3 * A^2;
            return I + ((1-cos(theta))/theta2)*A + ((theta-sin(theta))/(theta2*theta))*A*A;

        case 2: 
            // 0.5*eye(3) - (1/theta^2) * (R - eye(3) - A - 0.5*A^2);
            // 0.5*eye(3) + (theta-sin(theta))/theta^3 * A + (2*(cos(theta)-1) + theta^2)/(2*theta^4) * A^2
            return 0.5*I + (theta-sin(theta))/(theta2*theta)*A + (theta2 + 2*cos(theta)-2)/(2*theta2*theta2)*A*A;

        default: // General case 
            Eigen::Matrix3d R = I + (sin(theta)/theta)*A + ((1-cos(theta))/theta2)*A*A;
            Eigen::Matrix3d S = I;
            Eigen::Matrix3d Ak = I;
            long int kfactorial = 1;
            for (int k=1; k<=m; ++k) {
                kfactorial = kfactorial*k;
                Ak = (Ak*A).eval();
                S = (S + (1.0/kfactorial)*Ak).eval();
            }
            if (m==0) { 
                return R;
            } else if (m%2){ // odd 
                return (1.0/kfactorial)*I + (pow(-1,(m+1)/2)/pow(theta,m+1))*A * (R - S);
            } else { // even
                return (1.0/kfactorial)*I + (pow(-1,m/2)/pow(theta,m)) * (R - S);
            }
    }
}


Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w) {
    // Computes the vectorized exponential map for SO(3)
    return Gamma_SO3(w, 0);
}


Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v) {
    // Computes the vectorized exponential map for SE_K(3)
    int K = (v.size()-3)/3;
    Eigen::MatrixXd X = Eigen::MatrixXd::Identity(3+K,3+K);
    Eigen::Matrix3d R;
    Eigen::Matrix3d Jl;
    Eigen::Vector3d w = v.head(3);
    double theta = w.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    if (theta < TOLERANCE) {
        R = I;
        Jl = I;
    } else {
        Eigen::Matrix3d A = skew(w);
        double theta2 = theta*theta;
        double stheta = sin(theta);
        double ctheta = cos(theta);
        double oneMinusCosTheta2 = (1-ctheta)/(theta2);
        Eigen::Matrix3d A2 = A*A;
        R =  I + (stheta/theta)*A + oneMinusCosTheta2*A2;
        Jl = I + oneMinusCosTheta2*A + ((theta-stheta)/(theta2*theta))*A2;
    }
    X.template block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        X.template block<3,1>(0,3+i) = Jl * v.template segment<3>(3+3*i);
    }
    return X;
}


Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X) {
    // Compute Adjoint(X) for X in SE_K(3)
    int K = X.cols()-3;
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(3+3*K, 3+3*K);
    // Eigen::Matrix3d R = X.template block<3,3>(0,0);
    Eigen::Matrix3d R = X.template block<3,3>(0,0);
    Adj.template block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        Adj.template block<3,3>(3+3*i,3+3*i) = R;
        Eigen::Vector3d p = X.template block<3,1>(0,3+i);
        Adj.template block<3,3>(3+3*i,0) = skew(p)*R;
    }
    return Adj;
}


void removeRowAndColumn(Eigen::MatrixXd& M, int index) {
    unsigned int dimX = M.cols();
    // cout << "Removing index: " << index<< endl;
    M.template block(index,0,dimX-index-1,dimX) = M.bottomRows(dimX-index-1).eval();
    M.template block(0,index,dimX,dimX-index-1) = M.rightCols(dimX-index-1).eval();
    M.conservativeResize(dimX-1,dimX-1);
}

} // namespace go2_estimators
} // namespace legged_software
/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.h
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#pragma once
#include <Eigen/Dense>
#include <iostream>

namespace legged_software {
namespace go2_estimators {

extern const double TOLERANCE;

long int factorial(int n);


Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, int n);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v);
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

} // namespace go2_estimators
} // namespace legged_software
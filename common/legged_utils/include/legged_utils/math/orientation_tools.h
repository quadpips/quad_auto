/*! @file orientation_tools.h
 *  @brief Utility functions for 3D rotations
 *
 *  This file contains rotation utilities.  We generally use "coordinate
 * transformations" as opposed to the displacement transformations that are
 * commonly found in graphics.  To describe the orientation of a body, we use a
 * rotation matrix which transforms from world to body coordinates. This is the
 * transpose of the matrix which would rotate the body itself into the correct
 * orientation.
 *
 *  This follows the convention of Roy Featherstone's excellent book, Rigid Body
 * Dynamics Algorithms and the spatial_v2 MATLAB library that comes with it.
 * Note that we don't use the spatial_v2 convention for quaternions!
 */

#pragma once

#include <cmath>
#include <iostream>
#include <type_traits>
#include <Eigen/Dense>

namespace legged_software {
namespace ori {

enum class CoordinateAxis { X, Y, Z };

/*!
 * Convert radians to degrees
 */
inline double rad2deg(double rad) 
{
  return rad * 180.0 / M_PI;
}

/*!
 * Convert degrees to radians
 */
inline double deg2rad(double deg) 
{
  return deg * M_PI / 180.0;
}


/*!
 * Compute rotation matrix for coordinate transformation. Note that
 * coordinateRotation(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
 * this transforms into a frame rotated by .1 radians!.
 */
inline Eigen::Matrix3d coordinateRotation(CoordinateAxis axis, double theta) 
{
  // static_assert(std::is_floating_point<double>::value,
                // "must use floating point value");
  double s = std::sin(theta);
  double c = std::cos(theta);

  Eigen::Matrix3d R;

  if (axis == CoordinateAxis::X) 
  {
    R << 1, 0, 0, 0, c, s, 0, -s, c;
  } else if (axis == CoordinateAxis::Y) 
  {
    R << c, 0, -s, 0, 1, 0, s, 0, c;
  } else if (axis == CoordinateAxis::Z) 
  {
    R << c, s, 0, -s, c, 0, 0, 0, 1;
  }

  return R;
}

inline Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& v) 
{
  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 3,
    // "must have 3x1 vector");

  Eigen::Matrix3d m;
  m << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  return m;
}

/*!
 * Go from rpy to rotation matrix.
 */
inline Eigen::Matrix3d rpyToRotMat(const Eigen::Vector3d& v) 
{
  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 3,
                // "must have 3x1 vector");
  Eigen::Matrix3d m = coordinateRotation(CoordinateAxis::X, v[0]) *
                      coordinateRotation(CoordinateAxis::Y, v[1]) *
                      coordinateRotation(CoordinateAxis::Z, v[2]);
  return m;
}




/*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix
 */
inline Eigen::Matrix3d vectorToSkewMat(const Eigen::Vector3d& v) 
{
  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 3,
  //               "Must have 3x1 matrix");
  Eigen::Matrix3d m;
  m << 0, -v[2], v[1], 
       v[2], 0, -v[0], 
       -v[1], v[0], 0;
  return m;
}

/*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector
 */
inline Eigen::Vector3d matToSkewVec(const Eigen::Matrix3d& m) 
{
  // static_assert(double::ColsAtCompileTime == 3 && double::RowsAtCompileTime == 3,
  //               "Must have 3x3 matrix");
  return 0.5 * Eigen::Vector3d(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0),
                                        (m(1, 0) - m(0, 1)));
}

/*!
 * Convert a coordinate transformation matrix to an orientation quaternion.
 */
inline Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& r1) 
{
  // static_assert(double::ColsAtCompileTime == 3 && double::RowsAtCompileTime == 3,
                // "Must have 3x3 matrix");
  Eigen::Quaterniond q;
  Eigen::Matrix3d r = r1.transpose();
  double tr = r.trace();
  if (tr > 0.0) 
  {
    double S = sqrt(tr + 1.0) * 2.0;
    q.w() = 0.25 * S;
    q.x() = (r(2, 1) - r(1, 2)) / S;
    q.y() = (r(0, 2) - r(2, 0)) / S;
    q.z() = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    double S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q.w() = (r(2, 1) - r(1, 2)) / S;
    q.x() = 0.25 * S;
    q.y() = (r(0, 1) + r(1, 0)) / S;
    q.z() = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    double S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q.w() = (r(0, 2) - r(2, 0)) / S;
    q.x() = (r(0, 1) + r(1, 0)) / S;
    q.y() = 0.25 * S;
    q.z() = (r(1, 2) + r(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q.w() = (r(1, 0) - r(0, 1)) / S;
    q.x() = (r(0, 2) + r(2, 0)) / S;
    q.y() = (r(1, 2) + r(2, 1)) / S;
    q.z() = 0.25 * S;
  }
  return q;
}

/*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 */
inline Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q) 
{
  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 4,
                // "Must have 4x1 quat");
  double e0 = q.w();
  double e1 = q.x();
  double e2 = q.y();
  double e3 = q.z();

  Eigen::Matrix3d R;

  R <<
    1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2),
      2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
      2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2);
  R.transposeInPlace();

  // USE XYZ ORDER
  // Eigen::Quaternion quat(q[0], q[1], q[2], q[3]);
  // double R = quat.toRotationMatrix();
  return R;
}

/*!
 * Square a number
 */
inline double square(double a) {
  return a * a;
}

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
inline Eigen::Vector3d quatToRPY(const Eigen::Quaterniond& q) // order: w, x, y, z
{
  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 4,
  //               "Must have 4x1 quat");
  Eigen::Vector3d rpy;
  double as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  rpy.z() =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                 square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  rpy.y() = std::asin(as);
  rpy.x() =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                 square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));

  return rpy;
}

inline void makeEulerAnglesUnique(Eigen::Vector3d& eulerAngles) 
{
  double tol(1e-9);  // FIXME(jcarius) magic number
  double pi(M_PI);

  if (eulerAngles.y() < -pi / 2 - tol) 
  {
    if (eulerAngles.x() < 0) 
    {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else 
    {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    eulerAngles.y() = -(eulerAngles.y() + pi);

    if (eulerAngles.z() < 0) 
    {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else 
    {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  } else if (-pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= -pi / 2 + tol) 
  {
    eulerAngles.x() -= eulerAngles.z();
    eulerAngles.z() = 0;
  } else if (-pi / 2 + tol < eulerAngles.y() && eulerAngles.y() < pi / 2 - tol) 
  {
    // ok
  } else if (pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= pi / 2 + tol) 
  {
    // todo: pi/2 should not be in range, other formula?
    eulerAngles.x() += eulerAngles.z();
    eulerAngles.z() = 0;
  } else  // pi/2 + tol < eulerAngles.y()
  {
    if (eulerAngles.x() < 0) 
    {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    eulerAngles.y() = -(eulerAngles.y() - pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  }
}

inline double scalarMod(double x, double y) 
{
    return fmodf(x, y);
}

// TODO: Add documentation
inline double moduloAngleWithReference(const Eigen::Vector3d& R, const double & reference) // , const T2 & 
{
  // std::cout << "[moduleoAngleWithReference] R: " << R.transpose() << ", reference: " << reference << std::endl;

  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 3,
  //               "Must have 3x1 vec");

  // const double reference = 0.0; // reference; // TODO: Add reference parameter
  const double ub = reference + M_PI;  // upper bound
  const double lb = reference - M_PI;  // lower bound

  double x = R[2]; // Assuming R is a vector with the angle in the third component
  double pi(M_PI);
  double two_pi(2.0 * M_PI);

  // unwind
  while (x > ub) 
  {
    x = lb + scalarMod(x - lb, two_pi);
  }
  
  while (x < lb) 
  {
    x = ub - scalarMod(ub - x, two_pi);
  }

  // std::cout << "[moduleoAngleWithReference] x: " << x << std::endl;

  return x;
}

/**
* @brief Convert a 3x3 rotation matrix to ocs2-style rpy euler angles, assume last yaw angle
*
* @param R : rotation matrix
* @param lastYaw : previous yaw angle
* @return Eigen::Vector3f : roll-pitch-yaw euler angles
*/
inline Eigen::Vector3d mat2oc2rpy(const Eigen::Matrix3d& R, const double & lastYaw) // order: w, x, y, z
{
  // https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/TerrainAdaptation.cpp#L19
  // https://github.com/lnotspotl/tbai/blob/main/tbai_core/include/tbai_core/Rotations.hpp
    
  Eigen::Vector3d eulerXYZ = R.eulerAngles(0, 1, 2);
  makeEulerAnglesUnique(eulerXYZ);
  double moduloYaw = moduloAngleWithReference(eulerXYZ, lastYaw);
  eulerXYZ.z() = moduloYaw;
  return eulerXYZ;
}

inline Eigen::Vector3d ocs2quatToRPY(const Eigen::Quaterniond& q, const double & lastYaw) // order: w, x, y, z
{
  // Base orientation - Euler xyz
  const Eigen::Quaterniond baseQuaternion = q; // w, x, y, z
  const Eigen::Matrix3d R_world_base = baseQuaternion.toRotationMatrix();
  const Eigen::Matrix3d R_base_world = R_world_base.transpose();
  const Eigen::Vector3d rpy = mat2oc2rpy(R_world_base, lastYaw);
  // lastYaw_ = rpy[2];

  return rpy; // Return roll, pitch, yaw

  // ROS_INFO_STREAM("                                       euler angles): ");
  // ROS_INFO_STREAM("                                         roll: " << rpy(0));
  // ROS_INFO_STREAM("                                         pitch: " << rpy(1));
  // ROS_INFO_STREAM("                                         yaw: " << rpy(2));
}

inline Eigen::Quaterniond rpyToQuat(const Eigen::Vector3d& rpy) 
{
  // static_assert(double::ColsAtCompileTime == 1 && double::RowsAtCompileTime == 3,
  //               "Must have 3x1 vec");
  Eigen::Matrix3d R = rpyToRotMat(rpy);
  Eigen::Quaterniond q = rotationMatrixToQuaternion(R);
  return q;
}

inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) 
{
  // static_assert(double::ColsAtCompileTime == 3 && double::RowsAtCompileTime == 3,
  //               "Must have 3x3 matrix");
  Eigen::Quaterniond q = rotationMatrixToQuaternion(R);
  Eigen::Vector3d rpy = quatToRPY(q);
  return rpy;
}

/*!
 * Take the product of two quaternions
 */
inline Eigen::Quaterniond quatProduct(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) 
{
  double r1 = q1.w();
  double r2 = q2.w();
  Eigen::Vector3d v1(q1.x(), q1.y(), q1.z());
  Eigen::Vector3d v2(q2.x(), q2.y(), q2.z());

  double r = r1 * r2 - v1.dot(v2);
  Eigen::Vector3d v = r1 * v2 + r2 * v1 + v1.cross(v2);
  Eigen::Quaterniond q(r, v[0], v[1], v[2]);
  return q;
}

inline void quaternionToso3(const Eigen::Quaterniond & quat, Eigen::Vector3d& so3) 
{
  so3[0] = quat.x();
  so3[1] = quat.y();
  so3[2] = quat.z();

  double theta = 2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));

  if (fabs(theta) < 0.0000001) 
  {
    so3.setZero();
    return;
  }
  so3 /= sin(theta / 2.0);
  so3 *= theta;
}

} // namespace orientation_tools
} // namespace legged_software

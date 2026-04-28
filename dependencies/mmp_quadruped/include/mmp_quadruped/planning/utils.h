// #ifndef UTILS_H
// #define UTILS_H
#pragma once

// #include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>

#include <string>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// #include <ocs2_quadruped/common/Types.h>
// #include <mmp_quadruped/MmpDimensions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <convex_plane_decomposition/PlanarRegion.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include "ocs2_switched_model_interface/core/Rotations.h"

// Include transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define TRANSITION_DISCR_FACTOR 2

namespace mmp {

using namespace switched_model;
using quaternion_t = Eigen::Quaternion<scalar_t>;

enum LegIndex {FL = 0, FR = 1, BL = 2, BR = 3}; /**< Enum for leg indices */

enum StepLabels {OOB = -1, VOID = 0, NONPASS = 1, PASS = 2, STEP = 3}; /**< Enum for steppability labels */

const std::map<int, double> StepLabelPenalties = {  {OOB, 5.0}, 
                                                    {VOID, 1000.0}, 
                                                    {NONPASS, 1000.0}, 
                                                    {PASS, 100.0}, 
                                                    {STEP, 1.0}  }; /**< Penalty values for steppability labels */

const std::map<int, std::string> LegName = { {FL, "Front left"}, 
                                             {FR, "Front right"}, 
                                             {BL, "Back left"}, 
                                             {BR, "Back right"}}; /**< Enum for full leg names, mostly for pretty-printing */

const std::map<int, std::string> LegNameShort = { {-1, "--"}, 
                                                  {FL, "FL"}, 
                                                  {FR, "FR"}, 
                                                  {BL, "BL"}, 
                                                  {BR, "BR"}}; /**< Enum for short leg names, mostly for pretty-printing */

const std::map<int, std::string> LegModeScheduleShort = { {-1, "--"}, 
                                                          {FL, "LF"}, 
                                                          {FR, "RF"}, 
                                                          {BL, "LH"}, 
                                                          {BR, "RH"}}; /**< Enum for short leg names that reflect Mode Schedule notation */

const std::map<int, std::string> FootLinks = {{FL, "FL_foot"}, 
                                              {FR, "FR_foot"}, 
                                              {BL, "RL_foot"}, 
                                              {BR, "RR_foot"}}; /**< Enum for foot link names that reflect URDF notation */

// const double LOCAL_GS_RADIUS = LOCAL_ENV_RADIUS - 0.10; /**< Radius in meters within which we perform local graph search */

const double Z_TORSO_OFFSET = 0.325; /**< Nominal offset from feet that we set the torso height, TODO: load in from comHeight */

const double FOOT_RADIUS = 0.02; /**< Radius of feet */

inline vector3_t extractTorsoPosition(const base_coordinate_t & torsoPose)
{
    assert (torsoPose.size() == 6); // for a 6D pose (position, orientation)

    return torsoPose.tail(3);
}

inline vector3_t extractTorsoOrientation(const base_coordinate_t & torsoPose)
{
    assert (torsoPose.size() == 6); // for a 6D pose (position, orientation)

    return torsoPose.head(3);
}

/**
* @brief Calculate the norm of a 3D vector
*
* @param vec The 3D vector
* @return double : The norm of the vector
*/
inline double norm(const geometry_msgs::msg::Vector3 & vec)
{
    return std::sqrt (std::pow(vec.x, 2) + std::pow(vec.y, 2) + std::pow(vec.z, 2));
}

inline std::pair<vector3_t, vector3_t> calculateUnitNormsFromRegionPoints(const vector3_t & bot_left_pt, 
                                                                            const vector3_t & bot_right_pt, 
                                                                            const vector3_t & top_right_pt)
{
    // exploiting fact that region is a quadrilateral
    vector3_t top_left_pt = bot_left_pt + (top_right_pt - bot_right_pt);

    // calculate e0 and e1
    vector3_t vec0 = (bot_right_pt - bot_left_pt);
    vector3_t e0_from = vec0 / vec0.norm();

    vector3_t vec1 = (top_left_pt - bot_left_pt);
    vector3_t e1_from = vec1 / vec1.norm();

    return std::make_pair(e0_from, e1_from);
}

inline vector3_t eulerXYZFromRotationMatrix(const Eigen::Matrix3d& R, const float& referenceYaw) // ,  = 0.0 
{
    vector3_t eulerXYZ = R.eulerAngles(0, 1, 2);
    ocs2::makeEulerAnglesUnique(eulerXYZ);
    eulerXYZ.z() = ocs2::moduloAngleWithReference(eulerXYZ.z(), referenceYaw);
    return eulerXYZ;
}

inline Eigen::Matrix3d rotationMatrixFromEulerXYZ(const vector3_t& eulerXYZ)
{
    Eigen::Matrix3d R = rotationMatrixBaseToOrigin<scalar_t>(eulerXYZ);
    return R;
}

inline quaternion_t quaternionFromRotationMatrix(const Eigen::Matrix3d& R)
{
    quaternion_t q(R);
    return q;
}

inline quaternion_t quaternionFromEulerXYZ(const vector3_t& eulerXYZ)
{
    Eigen::Matrix3d R = rotationMatrixFromEulerXYZ(eulerXYZ);
    return quaternionFromRotationMatrix(R);
}

inline double quaternionAngularDiff(const quaternion_t & q1, const quaternion_t & q2)
{
    double dot_product = q1.dot(q2);
    dot_product = std::min(1.0, std::max(-1.0, dot_product)); // clamp to valid range for acos

    double angle_diff = 2 * std::acos(std::abs(dot_product)); // angle between quaternions

    return angle_diff;
}

/**
* @brief Calculate the 3D orientation from two orthonormal unit vectors (the A and the B in the cross product A x B)
*
* @param e0 The first unit vector
* @param e1 The second unit vector
* @return vector3_t : The orientation in roll, pitch, yaw
*/
inline vector3_t calculateOrientationFromUnitNorms(const vector3_t & e0, 
                                                    const vector3_t & e1,
                                                    const float& referenceYaw)
{
    // Taking from OCS2
    // float referenceYaw = 0.0; // lastYaw;

    Eigen::Matrix3d R;
    R.col(0) = e0;
    R.col(1) = e1;
    R.col(2) = e0.cross(e1);

    vector3_t eulerXYZ = eulerXYZFromRotationMatrix(R, referenceYaw);
    return eulerXYZ;   
}


// inline vector3_t calculateOrientationFromRegionPoints(const std::vector<vector3_t> & region_pts)
// {
//     std::pair<vector3_t, vector3_t> unitNorms = calculateUnitNormsFromRegionPoints(region_pts.at(0), 
//                                                                                                 region_pts.at(1), 
//                                                                                                 region_pts.at(2));
//     return calculateOrientationFromUnitNorms(unitNorms.first, unitNorms.second);
// }

/**
* @brief Estimate the normal of a plane from a set of points
* taken from: https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
*
* @param points The set of points
* @return vector3_t : The normal of the plane
*/
inline vector3_t estimatePlaneNormal(const std::vector<vector3_t> & points)
{
    int N = points.size();

    vector3_t sum(0.0, 0.0, 0.0);
    for (int i = 0; i < points.size(); i++)
        sum += points[i];
    
    vector3_t centroid = sum / points.size();

    double xx = 0.0; double xy = 0.0; double xz = 0.0;
    double yy = 0.0; double yz = 0.0;
    double zz = 0.0;

    vector3_t r(0.0, 0.0, 0.0);
    for (int i = 0; i < points.size(); i++)
    {
        r = points[i] - centroid;

        xx += (r[0] * r[0]);
        xy += (r[0] * r[1]);
        xz += (r[0] * r[2]);
        yy += (r[1] * r[1]);
        yz += (r[1] * r[2]);
        zz += (r[2] * r[2]);
    }

    double det_z = xx * yy - xy * xy;

    vector3_t unnormalized_normal(xy * yz - xz * yy,
                                        xy * xz - yz * xx,
                                        det_z);

    vector3_t normal = unnormalized_normal / unnormalized_normal.norm();

    // std::cout << "normal: " << normal.transpose() << std::endl;
    return normal; 
}

/** 
* @brief Estimate the torso orientation from a set of contact points
*
* @param points The set of contact points
* @return vector3_t : The torso orientation in roll, pitch, yaw
*/
inline vector3_t estimateEulerAnglesFromContacts(const std::vector<vector3_t> & points,
                                                    const float& referenceYaw)
{
    // std::cout << "[estimateEulerAnglesFromContacts()]" << std::endl;

    // Ensure that we have 4 points, one for each foot
    assert(points.size() == 4);

    vector3_t avgContactPosition = (points[0] + points[1] + points[2] + points[3]) / 4.0;

    vector3_t normal = estimatePlaneNormal(points);

    double D = avgContactPosition.dot(normal);

    vector3_t FLPoint = points[0];
    vector3_t inPlaneFLPoint(FLPoint[0], FLPoint[1], (D - normal[0]*FLPoint[0] - normal[1]*FLPoint[1]) / normal[2]);

    vector3_t FRPoint = points[1];
    vector3_t inPlaneFRPoint(FRPoint[0], FRPoint[1], (D - normal[0]*FRPoint[0] - normal[1]*FRPoint[1]) / normal[2]);

    vector3_t XDir = ((inPlaneFLPoint - avgContactPosition) + (inPlaneFRPoint  - avgContactPosition)) / 2.0; // guaranteed to be in plane as well
    vector3_t XUnitNorm = XDir / XDir.norm();

    vector3_t YUnitNorm = normal.cross(XUnitNorm);

    return calculateOrientationFromUnitNorms(XUnitNorm, YUnitNorm, referenceYaw);

    // if (points[0][0] == points[1][0] && points[2][0] == points[3][0] && points[0][0] > points[2][0] &&
    //     points[0][1] == points[2][1] && points[1][1] == points[3][1] && points[0][1] > points[1][1])
    //     // points[0][2] == points[1][2] && points[2][2] == points[3][2] && points[0][2] > points[2][2])
    // {
    //     std::cout << "      FL_point: " << points[0].transpose() << std::endl;
    //     std::cout << "      FR_point: " << points[1].transpose() << std::endl;
    //     std::cout << "      BL_point: " << points[2].transpose() << std::endl;
    //     std::cout << "      BR_point: " << points[3].transpose() << std::endl;
    //     std::cout << "      theta_x: " << theta_x << ", theta_y: " << theta_y << ", theta_z: " << theta_z << std::endl;
    // }

    // return vector3_t(theta_x, theta_y, theta_z);
}

/**
* @brief Calculate the angular distance between two 3D vectors
*
* @param vec1 The first 3D vector
* @param vec2 The second 3D vector
* @return double : The angular distance between the two vectors
*/
inline double vec2vecAngularDiff(const vector3_t & vec1, 
                                    const vector3_t & vec2)
{
    // std::cout << "[vec2vecAngularDiff()]" << std::endl;
    double eps = 0.00001;

    double dot = vec1.dot(vec2);
    double norms = vec1.norm() * vec2.norm();

    double arg = dot / norms;

    // std::cout << "      dot: " << dot << std::endl;
    // std::cout << "      norms: " << norms << std::endl;

    if (std::abs(1.0 - arg) < eps)
    {
        // std::cout << "          case 1" << std::endl;
        return 0.0;
    } else if (std::abs(-1.0 - arg) < eps)
    {
        // std::cout << "          case 2" << std::endl;
        return M_PI;
    }   

    double res = std::acos(arg);
    // std::cout << "          res: " << res << std::endl; 

    return res;
}

/**
* @brief Pretty-print the 18-dim configuration vector of the quadruped
*
* @param q The 18-dim configuration vector
* @return std::string : The pretty-printed string
*/
inline std::string configurationString(const Eigen::VectorXd & q)
{
    assert(q.size() == 18);

    std::string s = "Torso orientation: " + std::to_string(q[0]) + ", " + std::to_string(q[1]) + ", " + std::to_string(q[2]) + "\n";
    s += "Torso position: " + std::to_string(q[3]) + ", " + std::to_string(q[4]) + ", " + std::to_string(q[5]) + "\n";
    s += "FL leg: " + std::to_string(q[6]) + ", " + std::to_string(q[7]) + ", " + std::to_string(q[8]) + "\n";
    s += "FR leg: " + std::to_string(q[9]) + ", " + std::to_string(q[10]) + ", " + std::to_string(q[11]) + "\n";
    s += "BL leg: " + std::to_string(q[12]) + ", " + std::to_string(q[13]) + ", " + std::to_string(q[14]) + "\n";
    s += "BR leg: " + std::to_string(q[15]) + ", " + std::to_string(q[16]) + ", " + std::to_string(q[17]) + "\n";

    return s;
}

/**
* @brief Pretty-print the 24-dim configuration vector of the quadruped
*
* @param q The 24-dim configuration vector
* @return std::string : The pretty-printed string
*/
inline std::string stateString(const comkino_state_t & x)
{
    assert(x.size() == 24);

    std::string s = "Torso orientation: " + std::to_string(x[0]) + ", " + std::to_string(x[1]) + ", " + std::to_string(x[2]) + "\n";
    s += "Torso position: " + std::to_string(x[3]) + ", " + std::to_string(x[4]) + ", " + std::to_string(x[5]) + "\n";
    s += "Torso angular velocity: " + std::to_string(x[6]) + ", " + std::to_string(x[7]) + ", " + std::to_string(x[8]) + "\n";
    s += "Torso linear velocity: " + std::to_string(x[9]) + ", " + std::to_string(x[10]) + ", " + std::to_string(x[11]) + "\n";
    s += "FL leg: " + std::to_string(x[12]) + ", " + std::to_string(x[13]) + ", " + std::to_string(x[14]) + "\n";
    s += "FR leg: " + std::to_string(x[15]) + ", " + std::to_string(x[16]) + ", " + std::to_string(x[17]) + "\n";
    s += "BL leg: " + std::to_string(x[18]) + ", " + std::to_string(x[19]) + ", " + std::to_string(x[20]) + "\n";
    s += "BR leg: " + std::to_string(x[21]) + ", " + std::to_string(x[22]) + ", " + std::to_string(x[23]) + "\n";

    return s;
}

/**
* @brief Pretty-print a vector of shorts
*
* @param vec The vector of shorts
* @return std::string : The pretty-printed string
*/
inline std::string vec2string(const std::vector<short> & vec)
{
    std::string str = "";
    for (int i = 0; i < vec.size(); i++)
    {
        str += ((vec[i] < 0 ? "-" : std::to_string(vec[i])) + (i == (vec.size() - 1) ? "" : "_"));
    }
    return str;  
}

/**
* @brief Pretty-print a vector of doubles
*
* @param vec The vector of doubles
* @return std::string : The pretty-printed string
*/
inline std::string vec2string(const std::vector<double> & vec)
{
    std::string str = "";
    for (int i = 0; i < vec.size(); i++)
    {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << vec[i];
        str += (stream.str() + (i == (vec.size() - 1) ? "" : "_"));
    }
    return str;    
}

/**
* @brief Helper function to convert an integer index into a coparameter index tuple, used for indexing ModeMatrix during mode search
*
* @param tuple The coparameter index tuple to convert to
* @param value The integer index value to convert
* @param nDims The number of dimensions in the mode matrix
* @param expStepSize The step size for the discretized experience distribution
*/
inline void intToTuple(std::vector<short> & tuple, 
                        const double & value, 
                        const int & nDims, 
                        const double & expStepSize)
{
    tuple = std::vector<short>(nDims);
    for (int coparamIdx = 0; coparamIdx < tuple.size(); coparamIdx++)
    {
        double subtractVal = 0.0;
        for (int k = coparamIdx; k > 0; k--)
            subtractVal += (tuple[k - 1] * std::pow(expStepSize, nDims - k));
        tuple[coparamIdx] = int((value - subtractVal) / std::pow(expStepSize, nDims - coparamIdx - 1));
    }    
}

}  // namespace mmp

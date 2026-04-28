#pragma once

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

// Include transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const float DELTA = std::numeric_limits<float>::epsilon();

// Parameters
struct SuperpixelParams
{
    // Floor image parameters
    int k_c_ = 512; // Floor width (pixels)
    double v_fov_ = M_PI / 2.0; // Vertical field of view (radians)
    double v_offset_ = 0.0;
    double h_ = (v_fov_ / 2) - v_offset_; // Vertical angle from camera to floor (radians)

    // Dilation parameters
    int num_dilation_iterations_ = 0; // Number of dilation iterations
    int kernel_radius_ = 0; // Kernel size for dilation

    // Superpixel algorithm parameters
    int num_iterations_ = 10; // Number of iterations
    int num_superpixels_ = 200; // Desired number of approximately equally-sized superpixels
    int step_ = 0; // superpixel grid interval
    bool warm_start_ = false; // Warm start
    bool constraint_ = true; // Use constraint
    bool ransac_ = true; // Refine normals via RANSAC
    bool snapping_ = false; // Snap clusters to nearest actual pixel

    // Superpixel distance parameters
    double w_normal_ = 1.0; // Weighting parameter for normal similarity term
    double w_plane_dist_ = 1.0; // Weighting parameter for plane - position distance term
    double w_world_dist_ = 1.0; // Weighting parameter for world - position distance term
    double w_compact_ = 3.0; // Weighting parameter for compactness term

    // RANSAC parameters
    size_t ransac_K = 10; // number of points to sample
    int ransac_N = 25; // number of iterations
    double ransac_T = 0.01; // threshold    
};

inline bool isPixelInBounds(const int & k_c, 
                            const cv::Point & pixel)
{
    cv::Point center = cv::Point(k_c / 2, k_c / 2);

    if (cv::norm(center - pixel) > k_c / 2)
    {
        return false;
    }

    if (pixel.x < 0 || pixel.x >= k_c || pixel.y < 0 || pixel.y >= k_c)
    {
        return false;
    }

    return true;
}

inline bool isClusterCentroidValid(const std::vector<double> & center)
{
    cv::Point center_pixel = cv::Point(center[0], center[1]);
    float center_depth = center[2];

    float min_acceptable_depth = 0.0;
    float max_acceptable_depth = 1.0;
    if (center_depth < min_acceptable_depth || center_depth > max_acceptable_depth)
    {
        return false;
    }

    Eigen::Vector3d normal(center[3], center[4], center[5]);    

    Eigen::Vector3d ideal_normal(0, -1.0, 0);
    if ( std::abs( normal.dot(ideal_normal) ) < 0.90 )
    {
        // RCLCPP_WARN_STREAM(node_->get_logger(), "Passing depth check but failing normal check.");
        return false;
    }

    return true;
}


inline bool isDepthValid(const cv::Mat & depth_image, 
                            const cv::Point & pixel,
                            const int & k_c)
{
    //////////////////
    // IMAGE BOUNDS //
    //////////////////

    if (!isPixelInBounds(k_c, pixel))
    {
        return false;
    }

    ///////////
    // DEPTH //
    ///////////

    float depth = depth_image.at<float>(pixel.y, pixel.x);

    if (std::isnan(depth) || std::abs(depth) < 1e-6)
    {
        return false;
    }

    float min_acceptable_depth = 0.0;
    float max_acceptable_depth = 1.0;

    if (depth < min_acceptable_depth || depth > max_acceptable_depth)
    {
        return false;
    }

    return true;
}

inline bool isPixelValid(const cv::Mat & depth_image, 
                            // const cv::Mat & label_image,
                            const cv::Mat & normal_image,
                            const cv::Point & pixel,
                            const int & k_c)
{
    //////////////////
    // IMAGE BOUNDS //
    //////////////////

    if (!isPixelInBounds(k_c, pixel))
    {
        return false;
    }

    ///////////
    // DEPTH //
    ///////////

    float depth = depth_image.at<float>(pixel.y, pixel.x);

    if (std::isnan(depth) || std::abs(depth) < 1e-6)
    {
        return false;
    }

    float min_acceptable_depth = 0.0;
    float max_acceptable_depth = 1.0;

    if (depth < min_acceptable_depth || depth > max_acceptable_depth)
    {
        return false;
    }

    ///////////
    // LABEL //
    ///////////

    // uint8_t label = label_image.at<uint8_t>(pixel.y, pixel.x);

    // if (std::isnan(label) || label < 0)
    // {
    //     RCLCPP_WARN_STREAM(node_->get_logger(), "Passing depth check but failing label check.");
    //     return false;
    // }

    ////////////
    // NORMAL //
    ////////////

    cv::Vec3f normal = normal_image.at<cv::Vec3f>(pixel.y, pixel.x);

    if (std::isnan(normal[0]) || std::isnan(normal[1]) || std::isnan(normal[2]) ||
        cv::norm(normal) < DELTA)
    {
        // RCLCPP_WARN_STREAM(node_->get_logger(), "Passing depth check but failing normal check.");
        return false;
    }

    // only considering normals pointing upwards
    cv::Vec3f ideal_normal = cv::Vec3f(0, -1.0, 0);
    if ( std::abs( normal.dot(ideal_normal) ) < 0.90 )
    {
        // RCLCPP_WARN_STREAM(node_->get_logger(), "Passing depth check but failing normal check.");
        return false;
    }

    return true;
}

/**
* @brief Transform a 6D pose from world frame to base frame, 
* performs rotation + translation, stores full pose
* performs rotation + translation, stores full pose
* 
* @param source The 6D pose in world frame
* @param worldToBaseTransform The transform from world to base frame
* @return Eigen::VectorXd : The 6D pose in base frame
*/
inline Eigen::Vector3d transformHelperPointStamped(const Eigen::Vector3d & source_pos,
                                                    const geometry_msgs::msg::TransformStamped & egocanFrameToWorldFrame)
{
    // std::cout << "[transformHelperVector3Stamped()]" << std::endl;

    // std::cout << "  worldFrameToBaseFrameTransform: " << worldToBaseTransform << std::endl;

    geometry_msgs::msg::PointStamped sourceVector, destVector;

    sourceVector.header.stamp = egocanFrameToWorldFrame.header.stamp;
    sourceVector.header.frame_id = "egocan";
    sourceVector.point.x = source_pos[0];
    sourceVector.point.y = source_pos[1];
    sourceVector.point.z = source_pos[2];

    tf2::doTransform(sourceVector, destVector, egocanFrameToWorldFrame);

    Eigen::Vector3d dest = Eigen::Vector3d::Zero(); // source.size()

    dest[0] = destVector.point.x;
    dest[1] = destVector.point.y;
    dest[2] = destVector.point.z;

    return dest;
}

/**
* @brief Transform a 6D pose from world frame to base frame, 
* performs rotation + translation, stores full pose
* performs rotation + translation, stores full pose
* 
* @param source The 6D pose in world frame
* @param worldToBaseTransform The transform from world to base frame
* @return Eigen::VectorXd : The 6D pose in base frame
*/
inline Eigen::VectorXd transformHelperPoseStamped(const Eigen::Vector3d & source_pos,
                                                    const Eigen::Quaterniond & source_quat,
                                                    const geometry_msgs::msg::TransformStamped & egocanFrameToWorldFrame)
{
    // std::cout << "[transformHelperVector3Stamped()]" << std::endl;

    // std::cout << "  worldFrameToBaseFrameTransform: " << worldToBaseTransform << std::endl;

    geometry_msgs::msg::PoseStamped sourceVector, destVector;

    sourceVector.header.stamp = egocanFrameToWorldFrame.header.stamp;
    sourceVector.header.frame_id = "egocan";
    sourceVector.pose.position.x = source_pos[0];
    sourceVector.pose.position.y = source_pos[1];
    sourceVector.pose.position.z = source_pos[2];

    // euler to quat
    Eigen::Quaterniond q_source = source_quat;

    sourceVector.pose.orientation.x = q_source.x();
    sourceVector.pose.orientation.y = q_source.y();
    sourceVector.pose.orientation.z = q_source.z();
    sourceVector.pose.orientation.w = q_source.w();
    // std::cout << "  sourceVector: " << sourceVector << std::endl;

    tf2::doTransform(sourceVector, destVector, egocanFrameToWorldFrame);

    // std::cout << "  destVector: " << destVector << std::endl;

    tf2::Quaternion q(destVector.pose.orientation.x,
                        destVector.pose.orientation.y,
                        destVector.pose.orientation.z,
                        destVector.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

    Eigen::VectorXd dest = Eigen::VectorXd::Zero(6); // source.size()

    dest[0] = destVector.pose.position.x;
    dest[1] = destVector.pose.position.y;
    dest[2] = destVector.pose.position.z;
    dest[3] = roll; // eulers_dest[0]; // yaw
    dest[4] = pitch; // eulers_dest[1]; // pitch
    dest[5] = yaw; // eulers_dest[2]; // roll

    return dest;
}

/**
* @brief Calculate the rotation matrix from roll, pitch, and yaw
*
* @param roll The roll angle
* @param pitch The pitch angle
* @param yaw The yaw angle
* @return Eigen::Matrix3d : The rotation matrix
*/
inline Eigen::Matrix3d calculateRotationMatrix(const double & roll, 
                                                const double & pitch, 
                                                const double & yaw)
{
    Eigen::Matrix3d rotMat;

    double R11 = std::cos(yaw)*std::cos(pitch);
    double R12 = std::cos(yaw)*std::sin(pitch)*std::sin(roll)-std::sin(yaw)*std::cos(roll);
    double R13 = std::cos(yaw)*std::sin(pitch)*std::cos(roll)+std::sin(yaw)*std::sin(roll);
    double R21 = std::sin(yaw)*std::cos(pitch);
    double R22 = std::sin(yaw)*std::sin(pitch)*std::sin(roll)+std::cos(yaw)*std::cos(roll);
    double R23 = std::sin(yaw)*std::sin(pitch)*std::sin(roll)-std::cos(yaw)*std::sin(roll);
    double R31 = -std::sin(pitch);
    double R32 = std::cos(pitch)*std::sin(roll);
    double R33 = std::cos(pitch)*std::cos(roll);

    rotMat << R11, R12, R13,
              R21, R22, R23,
              R31, R32, R33;

    return rotMat;    
}

inline void pixelToEgocanFrame(cv::Vec3f & egocanPt,
                                const cv::Point & pixel,
                                const float & depth,
                                const int & k_c,
                                const double & h)
{
    egocanPt[0] = (pixel.x - (k_c / 2)) * (depth * 2 / (h * k_c));
    egocanPt[1] = depth;
    egocanPt[2] = (pixel.y - (k_c / 2)) * (depth * 2 / (h * k_c));
}
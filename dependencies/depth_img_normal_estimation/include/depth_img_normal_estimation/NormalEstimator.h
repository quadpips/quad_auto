#pragma once

// #include <chrono>
#include <fstream>
#include <string>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/image.hpp>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>

#include <yaml-cpp/yaml.h>

#include <mutex>

#include <depth_img_normal_estimation/PinholeCamera.h>

class NormalEstimator
{
    public:

        NormalEstimator(const rclcpp::Node::SharedPtr & nodePtr,
                        const std::string & camera_depth_topic, 
                        const std::string & config_path,
                        const bool & hardware);

        // ~NormalEstimator();

        void runNormalEstimation();

    private:

        // void log();

        void checkSparsity(const cv::Mat& depth_img, cv_bridge::CvImagePtr& normals_ptr);

        void depthImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

        void estimateNormals(const cv::Mat& depth_img, cv_bridge::CvImagePtr& normals);

        void publishNormals(const cv_bridge::CvImagePtr& normals, const cv_bridge::CvImagePtr& normals_bgr);

        bool notReceivedDepthImage();

        image_transport::Subscriber depth_img_sub;
        image_transport::Publisher normals_pub; /**< estimated normals image publisher */
        image_transport::Publisher normals_bgr_img_pub; /**< estimated normals image publisher */
        image_transport::Publisher filtered_depth_pub; /**< filtered depth image publisher */

        std::mutex depth_img_mutex;
        // ros::NodeHandle nodeHandle;
        rclcpp::Node::SharedPtr nodePtr_;

        cv::Mat depth_img_padded;

        cv_bridge::CvImagePtr depth_img_ptr;
        cv_bridge::CvImagePtr filtered_depth_ptr;
        cv_bridge::CvImagePtr normals_ptr;
        cv_bridge::CvImagePtr normals_bgr_ptr;

        bool hardware_;

        PinholeCamera camera;

        // std::chrono::steady_clock::time_point initTime;

        // std::chrono::steady_clock::time_point totalBegin;
        // std::chrono::steady_clock::time_point totalEnd;
        // float totalTimeTaken = 0.0f;
        // int numberOfTotalCalls = 0;

        // std::chrono::steady_clock::time_point preprocessBegin;
        // std::chrono::steady_clock::time_point preprocessEnd;
        // float preprocessTimeTaken = 0.0f;
        // int numberOfPreprocessCalls = 0;

        // std::chrono::steady_clock::time_point paddingBegin;
        // std::chrono::steady_clock::time_point paddingEnd;
        // float paddingTimeTaken = 0.0f;
        // int numberOfPaddingCalls = 0;

        // std::chrono::steady_clock::time_point depthGradientsBegin;
        // std::chrono::steady_clock::time_point depthGradientsEnd;
        // float depthGradientsTimeTaken = 0.0f;
        // int numberOfDepthGradientsCalls = 0;

        struct NormalEstimationParams
        {
            float depth_thresh;

            int bilat_filter_num_iters;
            int bilat_filter_kernel_size;
            float bilat_filter_sigma_color;
            float bilat_filter_sigma_space;

            int infill_filter_kernel_size;
        };

        NormalEstimationParams params;
};
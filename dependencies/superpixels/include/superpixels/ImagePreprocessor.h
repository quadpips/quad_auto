#pragma once

#include <superpixels/utils.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Include transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <random>

class ImagePreprocessor
{
    public:
        ImagePreprocessor(const SuperpixelParams & params, const rclcpp::Node::SharedPtr & nodePtr);

        void setParams(const SuperpixelParams & params);

        void cleanImages(const cv::Mat & raw_depth_img,
                            const cv::Mat & raw_normal_img,
                            cv::Mat & checked_depth_img,
                            cv::Mat & checked_normal_img,
                            cv::Mat & visited);

        void healthCheck(const cv::Mat & depth_img,
                            const cv::Mat & normal_img);

        void preprocessImages(const cv::Mat & checked_depth_img,
                                const cv::Mat & checked_normal_img,
                                cv::Mat & preprocessed_depth_img,
                                cv::Mat & preprocessed_normal_img);

        // const cv_bridge::CvImagePtr & raw_depth_img_ptr
        void fillInImage(const cv::Mat & cleaned_depth_img,
                            const cv::Mat & cleaned_normal_img,
                            const cv::Mat & visited,
                            cv::Mat & filled_depth_img,
                            cv::Mat & filled_normal_img,
                            const double & default_height);

    private:
        SuperpixelParams params_;

        // std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
        // std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

        std::default_random_engine generator;

        rclcpp::Node::SharedPtr node_;
};
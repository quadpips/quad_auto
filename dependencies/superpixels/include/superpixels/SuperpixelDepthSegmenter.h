#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <image_transport/subscriber_filter.hpp>
#include <tf2_ros/message_filter.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <math.h>

// #include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// #include <dynamic_reconfigure/server.h>
// #include <superpixels/ParametersConfig.h>

// #include <superpixels/utils.h>
#include <superpixels/ConvexHullifier.h>
#include <superpixels/ImagePreprocessor.h>
#include <superpixels/Visualizer.h>
#include <superpixels/Ransac.h>

using namespace std::placeholders;

class SuperpixelDepthSegmenter 
{
    public:
        SuperpixelDepthSegmenter(const rclcpp::Node::SharedPtr & node); // , const std::string & config_path
        ~SuperpixelDepthSegmenter();

        /////////////
        // SEGMENT //
        /////////////
        void run();

        void log();

        void visualize();

    private:
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        void reset_data(const cv::Mat & depth_image,
                        // const cv::Mat & label_image,
                        const cv::Mat & normal_image);

        void init_data(const cv::Mat & depth_image,
                        // const cv::Mat & label_image,
                        const cv::Mat & normal_image);

        cv::Point findLocalMinimum(const cv::Mat & depth_image, 
                                    // const cv::Mat & label_image,
                                    const cv::Mat & normal_image,
                                    const cv::Point & og_center);

        void generateSuperpixels(const cv::Mat & depth_image,
                                    // const cv::Mat & label_image,
                                    const cv::Mat & normal_image);

        bool checkConstraints(const int & center_idx, 
                                const float & depth,
                                // const uint8_t & label,
                                const cv::Vec3f & normal,
                                const cv::Point & pixel);

        double computeDistance(const int & center_idx, 
                                const float & depth,
                                // const uint8_t & label,
                                const cv::Vec3f & normal,
                                const cv::Point & pixel);

        cv::Vec3f ransac(const std::vector<cv::Point> & pixels, const cv::Mat & depth_image, const cv::Vec3f & og_normal);

        // void dilate_img(const cv::Mat & image, cv::Mat & dilated_image);

        // void checkSparsity();

        bool notReceivedImage();

        bool notReceivedDepthImage();

        // bool notReceivedLabelImage();

        bool notReceivedNormalImage();

        void allImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg, 
                                // const sensor_msgs::ImageConstPtr& label_image_msg, 
                                const sensor_msgs::msg::Image::ConstSharedPtr& normal_image_ms);

        cv::Point findCentroid(const cv::Mat & depth_image, 
                                // const cv::Mat & label_image,
                                const cv::Mat & normal_image,
                                const cv::Point & og_center);

        cv::Point findClosestPixel(const int & center_idx,
                                    const cv::Point & center, 
                                    const cv::Mat & depth_image,
                                    // const cv::Mat & label_image,
                                    const cv::Mat & normal_image);

        // ros::NodeHandle nh_;
        rclcpp::Node::SharedPtr node_;
        
        // Subscribers

        image_transport::SubscriberFilter raw_depth_img_sub_; 
        // image_transport::SubscriberFilter raw_label_img_sub_;
        image_transport::SubscriberFilter raw_normal_img_sub_;

        // Image message pointers
        sensor_msgs::msg::Image::ConstSharedPtr raw_depth_img_msg_ = nullptr;
        // sensor_msgs::ImageConstPtr raw_label_img_msg_ = nullptr;
        sensor_msgs::msg::Image::ConstSharedPtr raw_normal_img_msg_ = nullptr;

        std::string depth_img_topic_ = "";
        // std::string label_img_topic_;
        std::string normal_img_topic_ = "";

        // Image pointers
        cv_bridge::CvImagePtr raw_depth_img_ptr_ = nullptr;
        // cv_bridge::CvImagePtr raw_label_img_ptr_ = nullptr;
        cv_bridge::CvImagePtr raw_normal_img_ptr_ = nullptr;
        cv_bridge::CvImagePtr prop_depth_img_ptr_ = nullptr;
        // cv_bridge::CvImagePtr prop_label_img_ptr_ = nullptr;
        cv_bridge::CvImagePtr prop_normal_img_ptr_ = nullptr;

        // Synchronizer
        using MsgSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>; // sensor_msgs::Image,   
        std::shared_ptr<MsgSynchronizer> msg_sync_ = nullptr;

        // Callback handle for parameter changes
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_ = nullptr;

        // Mutex
        std::mutex img_mutex_;

        // Flags
        bool initialized_ = false;

        cv::Mat visited_ = cv::Mat(); // Visited pixels

        // Superpixel matrices and vector
        cv::Mat clusters_ = cv::Mat(); // per-pixel cluster assignments
        cv::Mat distances_ = cv::Mat(); // per-pixel distances to cluster center
        std::vector<std::vector<double>> centers_ = {}; // LAB/xy cluster centers
        std::vector<int> center_counts_ = {}; // Number of occurrences of each center
        std::vector<std::vector<cv::Point>> superpixels_ = {}; // Superpixel pixel locations
        std::vector<std::vector<Eigen::Vector2d>> superpixel_projections_ = {}; // Superpixel projections
        std::vector<std::vector<Eigen::Vector2d>> superpixel_convex_hulls_ = {}; // Superpixel convex hulls
        std::vector<Eigen::Matrix3d> egocan_to_region_rotations_ = {}; // Superpixel rotations

        cv::Mat preprocessed_depth_img = cv::Mat();
        cv::Mat preprocessed_normal_img = cv::Mat(); // preprocessed_label_img, 

        SuperpixelParams params_;

        geometry_msgs::msg::TransformStamped egocanFrameToOdomFrame = geometry_msgs::msg::TransformStamped();

        std::chrono::steady_clock::time_point totalBegin, totalEnd;
        float totalTimeTaken = 0.0;
        int numberOfTotalCalls = 0;

        std::chrono::steady_clock::time_point cleanBegin, cleanEnd;
        float cleanTimeTaken = 0.0;
        int numberOfCleanCalls = 0;

        std::chrono::steady_clock::time_point fillBegin, fillEnd;
        float fillTimeTaken = 0.0;
        int numberOfFillCalls = 0;

        std::chrono::steady_clock::time_point preprocessBegin, preprocessEnd;
        float preprocessTimeTaken = 0.0;
        int numberOfPreprocessCalls = 0;

        std::chrono::steady_clock::time_point superpixelBegin, superpixelEnd;
        float superpixelTimeTaken = 0.0;
        int numberOfSuperpixelCalls = 0;

        std::chrono::steady_clock::time_point ransacBegin, ransacEnd;
        float ransacTimeTaken = 0.0;
        int numberOfRansacCalls = 0;

        std::chrono::steady_clock::time_point convexHullBegin, convexHullEnd;
        float convexHullTimeTaken = 0.0;
        int numberOfConvexHullCalls = 0;

        std::chrono::steady_clock::time_point initTime;

        // std::chrono::steady_clock::time_point regionSplitBegin, regionSplitEnd;
        // std::chrono::steady_clock::time_point visBegin, visEnd;

        std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tfBuffer_{nullptr};

        ImagePreprocessor * imagePreprocessor_ = nullptr;
        Visualizer * visualizer_ = nullptr;
        Ransac * ransac_ = nullptr;
        ConvexHullifier * convexHullifier_ = nullptr;
};
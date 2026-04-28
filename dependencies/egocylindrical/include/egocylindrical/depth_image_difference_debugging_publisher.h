#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_DEBUGGING_PUBLISHER_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_DEBUGGING_PUBLISHER_H

#include <egocylindrical/depth_image_difference.h>  //Strictly speaking, a forward declaration would suffice, but not worth the potential hassle
#include <rclcpp/rclcpp.hpp>
#include <map>

namespace egocylindrical
{
    namespace utils
    {
        struct DIDiffDebuggingPublisher
        {
            using func_type = std::function<void(DIDiffDebugging)>;
//             struct Impl
//             {
//                 ros::Publisher pub;
//                 func_type op_f;
//             };

            std::map<std::string, func_type> publishers_;

            // void init(ros::NodeHandle pnh);
            void init(const rclcpp::Node::SharedPtr& node);
            void publish(const DIDiffDebugging& info);
            void reset();

            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_diff_image_pub;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reproj_depth_image_pub;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr range_image_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dilated_range_image_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dilated_point_cloud_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;

        };
    }
}

#endif //egocylindrical_depth_image_difference_debugging_publisher_h

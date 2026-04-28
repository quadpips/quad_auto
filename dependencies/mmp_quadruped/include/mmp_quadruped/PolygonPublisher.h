#pragma once

// #include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.h>
// #include <jsoncpp/json/json.h>
#include <json/json.h>
#include <fstream>
#include <Eigen/Dense>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include "ocs2_mpc/SystemObservation.h"

#include <mmp_quadruped/planning/utils.h>


namespace mmp {

class PolygonPublisher
{
    public:
        PolygonPublisher(const rclcpp::Node::SharedPtr& node, 
                            const std::string & jsonFile);

        void publishLocalPolygons();

    private:
        visualization_msgs::msg::MarkerArray generateSteppingStoneArrayFromFile(const std::string& setup_path);

        // ros::Subscriber observationSub_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;

        ocs2::SystemObservation latestObservation_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_pub;
        std::string jsonFile_;
        comkino_state_t latest_x;   
        Json::Value json_file;
        rclcpp::Node::SharedPtr node_;

        bool readyToPublish_ = false;

};

}  // namespace mmp

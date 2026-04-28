//
// Created by root on 2/5/18.
//

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/thread/mutex.hpp>


namespace egocylindrical
{

    namespace utils
    {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr generate_projected_point_cloud(const utils::ECWrapper& points);
    }

class ProjectedPointCloudGenerator
{   using Mutex = boost::mutex;
    using Lock = Mutex::scoped_lock;
    
    // ros::NodeHandle nh_, pnh_;
    rclcpp::Node::SharedPtr node_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Subscription<egocylindrical_msgs::msg::EgoCylinderPoints>::SharedPtr ec_sub_;

    // Mutex connect_mutex_;
    
public:

    ProjectedPointCloudGenerator(rclcpp::Node::SharedPtr node);

    bool init();


private:
    
    void ssCB();

    void ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg);

};


}

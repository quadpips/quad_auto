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
        sensor_msgs::msg::PointCloud2::ConstSharedPtr generate_normal_point_cloud_floor_only(const utils::ECWrapper& points);
    }

class NormalPointCloudFloorOnlyGenerator
{   using Mutex = boost::mutex;
    using Lock = Mutex::scoped_lock;
    
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pc_pub_;
    ros::Subscriber ec_sub_;

    Mutex connect_mutex_;
    
public:

    NormalPointCloudFloorOnlyGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool init();


private:
    
    void ssCB();

    void ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg);

};


}

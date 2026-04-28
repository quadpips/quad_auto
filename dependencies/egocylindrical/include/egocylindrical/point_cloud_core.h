#ifndef EGOCYLINDRICAL_POINT_CLOUD_CORE_H
#define EGOCYLINDRICAL_POINT_CLOUD_CORE_H

#include <egocylindrical/ecwrapper.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace egocylindrical
{
    namespace utils
    {
        sensor_msgs::msg::PointCloud2::SharedPtr generate_point_cloud(const utils::ECWrapper& points);
    }

}

#endif  //EGOCYLINDRICAL_POINT_CLOUD_CORE_H
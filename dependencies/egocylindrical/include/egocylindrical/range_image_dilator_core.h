#ifndef EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_IMPL_H
#define EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_IMPL_H


//#include <egocylindrical/ecwrapper.h>
// #include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
// #include <image_transport/image_transport.hpp>
// #include <image_transport/subscriber_filter.hpp>
// #include <message_filters/subscriber.h>
//
// #include <message_filters/synchronizer.h>
// #include <message_filters/time_synchronizer.h>
//
// // #include <dynamic_reconfigure/server.h>

// #include <rclcpp/rclcpp.hpp>
// #include <boost/thread/mutex.hpp>

// #include <opencv2/imgproc.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace egocylindrical
{

namespace utils
{

sensor_msgs::msg::Image::SharedPtr dilateImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_in);
}
}

#endif //EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_IMPL_H

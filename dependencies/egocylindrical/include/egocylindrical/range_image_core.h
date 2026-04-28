#ifndef RANGE_IMAGE_CORE_H
#define RANGE_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/msg/image.hpp>


namespace egocylindrical
{
    
    namespace utils
    {
        sensor_msgs::msg::Image::SharedPtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        sensor_msgs::msg::Image::SharedPtr getRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);
    }
}

#endif //RANGE_IMAGE_CORE_H

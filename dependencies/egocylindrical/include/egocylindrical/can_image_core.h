#ifndef CAN_IMAGE_CORE_H
#define CAN_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/msg/image.hpp>


namespace egocylindrical
{
    
    namespace utils
    {
        sensor_msgs::msg::Image::SharedPtr getRawCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        sensor_msgs::msg::Image::SharedPtr getCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);
        
    }
}

#endif //CAN_IMAGE_CORE_H

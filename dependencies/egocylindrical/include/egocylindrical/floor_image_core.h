#ifndef FLOOR_IMAGE_CORE_H
#define FLOOR_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/msg/image.hpp>


namespace egocylindrical
{
    
    namespace utils
    {
        // normals
        sensor_msgs::msg::Image::SharedPtr getFloorNormalColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        sensor_msgs::msg::Image::SharedPtr getFloorNormalImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        // labels
        sensor_msgs::msg::Image::SharedPtr getFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        sensor_msgs::msg::Image::SharedPtr getFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        // depth
        sensor_msgs::msg::Image::SharedPtr getRawFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);

        sensor_msgs::msg::Image::SharedPtr getFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::msg::Image::SharedPtr preallocated_msg=nullptr);
        
    }
}

#endif //FLOOR_IMAGE_CORE_H

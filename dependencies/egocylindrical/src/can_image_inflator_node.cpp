#include <egocylindrical/can_image_inflator_generator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_can_image_inflator_publisher");
        
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::CanImageInflatorGenerator s(nh, pnh);
    s.init();
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    ros::spin();
}

#include <egocylindrical/range_image_dilator_generator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_range_image_dilator_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::RangeImageDilator s(nh, pnh);
    s.init();
    ros::spin();
}

#include <egocylindrical/range_image_inflator_generator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_range_image_inflator_publisher");
        
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::RangeImageInflatorGenerator s(nh, pnh);
    s.init();
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    ros::spin();
}

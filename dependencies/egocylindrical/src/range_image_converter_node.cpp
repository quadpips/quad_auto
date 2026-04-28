#include <egocylindrical/range_image_converter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_image_to_points_converter_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::RangeImageConverter s(nh, pnh);
    s.init();
    ros::spin();
}

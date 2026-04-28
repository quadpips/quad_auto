#include "dedicated_egocylindrical_propagator.cpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_pointcloud_publisher");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::DedicatedEgoCylindricalPropagator s(nh, pnh);
    s.init();
    ros::spin();
}

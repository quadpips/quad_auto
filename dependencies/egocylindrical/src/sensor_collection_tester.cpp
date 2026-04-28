#include <egocylindrical/sensor_collection.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_collection_tester");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    tf2_ros::Buffer buffer;
    
    std::string fixed_frame_id = "odom";
    
    auto seq_cb = [](egocylindrical::utils::SensorMeasurement::Ptr measurement)
    {
        // ROS_INFO_STREAM("Received message with header: " << measurement->header);
    };
    
    egocylindrical::utils::SensorCollection s(pnh, buffer);
    s.init(fixed_frame_id, seq_cb);
    ros::spin();
}

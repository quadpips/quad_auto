// #include <ros/init.h>
// #include <ros/node_handle.h>

#include <superpixels/SuperpixelDepthSegmenter.h>
#include <segmented_planes_terrain_model/SegmentedPlanesTerrainModelRos.h>

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodePtr = rclcpp::Node::make_shared("superpixel_depth_segmentation_node",
                                                                rclcpp::NodeOptions()
                                                                .allow_undeclared_parameters(true)
                                                                .automatically_declare_parameters_from_overrides(true));

    rclcpp::Rate loop_rate(30.0); // 30 Hz

    // std::string config_path;
    // config_path = nodePtr->get_parameter("config_path").as_string();

    // bool terrain_receiver = nodePtr->get_parameter("terrain_receiver").as_bool();

    SuperpixelDepthSegmenter superpixel_segmenter = SuperpixelDepthSegmenter(nodePtr); // , config_path

    // dynamic_reconfigure::Server<superpixels::ParametersConfig> server;
    // dynamic_reconfigure::Server<superpixels::ParametersConfig>::CallbackType serverCallback;

    // serverCallback = boost::bind(&SuperpixelDepthSegmenter::reconfigureCallback, superpixel_segmenter, _1, _2);
    // server.setCallback(serverCallback);
    
    // to visualize regions 
    // if (terrain_receiver)
    // {
    //     RCLCPP_INFO_STREAM(nodePtr->get_logger(), "Terrain receiver is enabled. Will visualize regions.");
    //     switched_model::SegmentedPlanesTerrainModelRos * terrain_model = new switched_model::SegmentedPlanesTerrainModelRos(nodePtr);

    // } else
    // {
    //     RCLCPP_INFO_STREAM(nodePtr->get_logger(), "Terrain receiver is disabled. Will not visualize regions.");
    // }

    while (rclcpp::ok())
    {
        // Do something
        superpixel_segmenter.run();

        // Visualize outputs
        superpixel_segmenter.visualize();

        // superpixel_segmenter.log();

        rclcpp::spin_some(nodePtr);
        loop_rate.sleep();
    }
    return 0;
}
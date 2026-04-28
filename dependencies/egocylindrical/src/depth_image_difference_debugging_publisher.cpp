#include <egocylindrical/depth_image_difference_debugging_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace egocylindrical
{
    namespace utils
    {

        void DIDiffDebuggingPublisher::init(const rclcpp::Node::SharedPtr& node) // ros::NodeHandle pnh)
        {
            // ros::NodeHandle ppnh(pnh, "debugging");

            // auto add_publisher = [this, &ppnh](std::string name, auto f)
            // {
            //     DIDiffDebugging d;
            //     using L = decltype(*f(d));

            //     ros::Publisher pub = ppnh.advertise<L>(name, 2);

            //     auto pub_func = [f,pub](DIDiffDebugging data)
            //     {
            //         auto msg = f(data);
            //         if(msg)
            //         {
            //             pub.publish(msg);
            //         }
            //     };

            //     publishers_.emplace(name, pub_func);
            // };

            // add_publisher("depth_image", [](DIDiffDebugging data){return data.depth_image;});
            // add_publisher("depth_diff_image", [](DIDiffDebugging data){return data.depth_diff_image;});
            // add_publisher("reprojected_depth_image", [](DIDiffDebugging data){return data.reproj_depth_image;});
            // add_publisher("range_image", [](DIDiffDebugging data){return data.range_image;});
            // add_publisher("point_cloud", [](DIDiffDebugging data){return data.point_cloud;});
            // add_publisher("dilated_range_image", [](DIDiffDebugging data){return data.dilated_range_image;});
            // add_publisher("dilated_point_cloud", [](DIDiffDebugging data){return data.dilated_point_cloud;});
            // add_publisher("marker_array", [](DIDiffDebugging data){return data.marker_array;});
            
            depth_image_pub = node->create_publisher<sensor_msgs::msg::Image>("depth_image", 2);
            depth_diff_image_pub = node->create_publisher<sensor_msgs::msg::Image>("depth_diff_image", 2);
            reproj_depth_image_pub = node->create_publisher<sensor_msgs::msg::Image>("reprojected_depth_image", 2);
            range_image_pub = node->create_publisher<sensor_msgs::msg::Image>("range_image", 2);
            point_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 2);
            dilated_range_image_pub = node->create_publisher<sensor_msgs::msg::Image>("dilated_range_image", 2);
            dilated_point_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("dilated_point_cloud", 2);
            marker_array_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 2);

            // &depth_diff_image_pub, &reproj_depth_image_pub,
                                        // &range_image_pub, 
            // , &point_cloud_pub, &dilated_range_image_pub, 
                // &dilated_point_cloud_pub, &marker_array_pub
            // auto add_publisher = [this, &point_cloud_pub, &depth_image_pub](std::string name, auto f)
            // {
            //     // DIDiffDebugging d;
            //     // using L = decltype(*f(d));

            //     // if (name == "depth_image")
            //     // {
            //     //     auto pub_func = [f,&depth_image_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) depth_image_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // } 
            //     // else if (name == "depth_diff_image")
            //     // {
            //     //     auto pub_func = [f,&depth_diff_image_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) depth_diff_image_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            //     // else if (name == "reprojected_depth_image")
            //     // {
            //     //     auto pub_func = [f,&reproj_depth_image_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) reproj_depth_image_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            //     // else if (name == "range_image")
            //     // {
            //     //     auto pub_func = [f,&range_image_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) range_image_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            //     // if (name == "point_cloud")
            //     // {
            //     //     auto pub_func = [f,&point_cloud_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) point_cloud_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            //     // else if (name == "dilated_range_image")
            //     // {
            //     //     auto pub_func = [f,&dilated_range_image_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) dilated_range_image_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            //     // else if (name == "dilated_point_cloud")
            //     // {
            //     //     auto pub_func = [f,&dilated_point_cloud_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) dilated_point_cloud_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            //     // else if (name == "marker_array")
            //     // {
            //     //     auto pub_func = [f,&marker_array_pub](DIDiffDebugging data)
            //     //     {
            //     //         auto msg = f(data);
            //     //         if (msg) marker_array_pub->publish(*msg);
            //     //     };
            //     //     publishers_.emplace(name, pub_func);
            //     // }
            // };
            
            // add_publisher("depth_image", [](DIDiffDebugging data){return data.depth_image;});
            // add_publisher("depth_diff_image", [](DIDiffDebugging data){return data.depth_diff_image;});
            // add_publisher("reprojected_depth_image", [](DIDiffDebugging data){return data.reproj_depth_image;});
            // add_publisher("range_image", [](DIDiffDebugging data){return data.range_image;});
            // add_publisher("point_cloud", [](DIDiffDebugging data){return data.point_cloud;});
            // add_publisher("dilated_range_image", [](DIDiffDebugging data){return data.dilated_range_image;});
            // add_publisher("dilated_point_cloud", [](DIDiffDebugging data){return data.dilated_point_cloud;});
            // add_publisher("marker_array", [](DIDiffDebugging data){return data.marker_array;}); 

        }

        void DIDiffDebuggingPublisher::publish(const DIDiffDebugging& data)
        {
            if (data.depth_image) depth_image_pub->publish(*data.depth_image);
            if (data.depth_diff_image) depth_diff_image_pub->publish(*data.depth_diff_image);
            if (data.reproj_depth_image) reproj_depth_image_pub->publish(*data.reproj_depth_image);
            if (data.range_image) range_image_pub->publish(*data.range_image);
            if (data.point_cloud) point_cloud_pub->publish(*data.point_cloud);
            if (data.dilated_range_image) dilated_range_image_pub->publish(*data.dilated_range_image);
            if (data.dilated_point_cloud) dilated_point_cloud_pub->publish(*data.dilated_point_cloud);
            if (data.marker_array) marker_array_pub->publish(*data.marker_array);

//             for(std::map<std::string, func_type>::iterator it = publishers_.begin(); it != publishers_.end(); ++it)
//             {
// //                 it->first
//                 it->second(data);
//             }
        }

        void DIDiffDebuggingPublisher::reset()
        {

        }

    }
}

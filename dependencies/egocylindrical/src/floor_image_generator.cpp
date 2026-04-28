//
// Created by root on 2/5/18.
//

#include <egocylindrical/floor_image_generator.h>
#include <egocylindrical/range_image_core.h>
#include <egocylindrical/floor_image_core.h>

using namespace std::chrono_literals;

namespace egocylindrical
{
    EgoCylinderFloorImageGenerator::EgoCylinderFloorImageGenerator(rclcpp::Node::SharedPtr node) :
        node_(node),
        it_(node_)
    {
        // std::cout<<"Egocylindrical Floor Image Node Constructed"<<std::endl;

        timer_ = node_->create_wall_timer(0.05s, std::bind(&EgoCylinderFloorImageGenerator::ssCB, this));

        initStartTime = std::chrono::steady_clock::now();
    }

    void EgoCylinderFloorImageGenerator::log()
    {
        std::ofstream logFile;
        logFile.open("/home/masselmeier3/Desktop/Research/quad_pips_experiments/timing/superpixels/floor_image/timing_log_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(initStartTime.time_since_epoch()).count()) + ".csv", std::ios::out);
        float averageTotalTime = totalTimeTaken * 1.0e-3 / static_cast<float>(numberOfTotalCalls);
        logFile << "avg total time (ms), number of calls" << std::endl;
        logFile << averageTotalTime << ", " << numberOfTotalCalls << std::endl;
        logFile.close();
    }
    
    bool EgoCylinderFloorImageGenerator::init()
    {
        use_raw_ = false;
        std::string floor_image_topic = "floor_image"; // image_topic = "image", 
        // std::string floor_labels_topic = "floor_labels";
        // std::string floor_labels_colored_topic = "floor_labels_colored";
        std::string floor_normals_topic = "floor_normals";
        std::string floor_normals_colored_topic = "floor_normals_colored";

        // auto parameters_and_prefixes = node_->list_parameters({}, 10);

        // for (auto & name : parameters_and_prefixes.names) {
        //     std::cout << "Parameter name: " << name << std::endl;
        // }
        // for (auto & prefix : parameters_and_prefixes.prefixes) {
        //     std::cout << "Parameter prefix: " << prefix << std::endl;
        // }

        // pnh_.getParam("use_raw", use_raw_ );
        node_->get_parameter("use_raw", use_raw_);
        
        // pnh_.getParam("image_topic", image_topic );
        // pnh_.getParam("floor_image_topic", floor_image_topic );
        // pnh_.getParam("floor_labels_topic", floor_labels_topic );
        // pnh_.getParam("floor_labels_colored_topic", floor_labels_colored_topic );
        // pnh_.getParam("floor_normals_topic", floor_normals_topic );
        // pnh_.getParam("floor_normals_colored_topic", floor_normals_colored_topic );
        node_->get_parameter("floor_image_topic", floor_image_topic);
        // node_->get_parameter("floor_labels_topic", floor_labels_topic);
        // node_->get_parameter("floor_labels_colored_topic", floor_labels_colored_topic);
        node_->get_parameter("floor_normals_topic", floor_normals_topic);
        node_->get_parameter("floor_normals_colored_topic", floor_normals_colored_topic);
        
        // reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        // reconfigure_server_->setCallback(std::bind(&EgoCylinderFloorImageGenerator::configCB, this, _1, _2));

        // image_transport::SubscriberStatusCallback image_cb = std::bind(&EgoCylinderFloorImageGenerator::ssCB, this);
        // {
        //     Lock lock(connect_mutex_);
            // im_pub_ = it_.advertise(image_topic, 2, image_cb, image_cb);
        floor_im_pub_ = it_.advertise(floor_image_topic, 2); // , image_cb, image_cb
        // labels_im_pub_ = it_.advertise(floor_labels_topic, 2); // , image_cb, image_cb
        // labels_colored_im_pub_ = it_.advertise(floor_labels_colored_topic, 2); // , image_cb, image_cb
        normals_im_pub_ = it_.advertise(floor_normals_topic, 2); // , image_cb, image_cb
        normals_colored_im_pub_ = it_.advertise(floor_normals_colored_topic, 2); // , image_cb, image_cb
        // }
        
        return true;
    }
    
    // void EgoCylinderFloorImageGenerator::configCB(const ConfigType &config, uint32_t level)
    // {
    //   //Num_threads not actually used right now, so not important to lock
    //   //WriteLock lock(config_mutex_);
      
    //   // ROS_INFO_STREAM("Updating Floor Image Generator config: num_threads=" << config.num_threads);
    //   num_threads_ = config.num_threads;
    // }
    
    void EgoCylinderFloorImageGenerator::ssCB()
    {
        //std::cout << (void*)ec_sub_ << ": " << im_pub_->get_subscription_count() << std::endl;
        // Lock lock(connect_mutex_);
        if (floor_im_pub_.getNumSubscribers()>0) // im_pub_->get_subscription_count()>0 || 
        {
            if(ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                // ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderFloorImageGenerator::ecPointsCB, this);
                ec_sub_ = node_->create_subscription<egocylindrical_msgs::msg::EgoCylinderPoints>(
                    "egocylindrical_points", 2, std::bind(&EgoCylinderFloorImageGenerator::ecPointsCB, this, std::placeholders::_1));
                // ROS_INFO("RangeImage Generator Subscribing");

            }
      
        }
        else
        {
            ec_sub_.reset();
            // ROS_INFO("RangeImage Generator Unsubscribing");

        }
    }

    
    void EgoCylinderFloorImageGenerator::ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg)
    {
        totalBegin = std::chrono::steady_clock::now();

        // ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[range_image_generator] Received [" << ec_msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
        
        
        // bool gen_range_image = im_pub_->get_subscription_count() > 0;
        bool gen_can_image = floor_im_pub_.getNumSubscribers() > 0;

        utils::ECWrapper ec_pts(ec_msg);
        
        if (gen_can_image)
        {
            // RANGE
            // ros::WallTime// start = ros::WallTime::now();
            
            sensor_msgs::msg::Image::ConstSharedPtr image_ptr = use_raw_ ? utils::getRawFloorImageMsg(ec_pts, num_threads_, preallocated_can_msg_) : utils::getFloorImageMsg(ec_pts, num_threads_, preallocated_can_msg_);
            
            // ROS_DEBUG_STREAM_NAMED("timing","Generating can image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                        
            // LABELS
            // start = ros::WallTime::now();

            // sensor_msgs::msg::Image::ConstSharedPtr labels_ptr = utils::getFloorLabelImageMsg(ec_pts, num_threads_, preallocated_labels_msg_);

            // ROS_DEBUG_STREAM_NAMED("timing","Generating can labels took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

            // LABELS COLORED
            // start = ros::WallTime::now();

            // sensor_msgs::msg::Image::ConstSharedPtr labels_colored_ptr = utils::getFloorLabelColoredImageMsg(ec_pts, num_threads_, preallocated_labels_colored_msg_);

            // ROS_DEBUG_STREAM_NAMED("timing","Generating can labels colored took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

            // NORMALS
            // start = ros::WallTime::now();

            sensor_msgs::msg::Image::ConstSharedPtr normals_ptr = utils::getFloorNormalImageMsg(ec_pts, num_threads_, preallocated_normals_msg_);

            // ROS_DEBUG_STREAM_NAMED("timing","Generating can normals took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

            // NORMALS COLORED
            // start = ros::WallTime::now();

            sensor_msgs::msg::Image::ConstSharedPtr normals_colored_ptr = utils::getFloorNormalColoredImageMsg(ec_pts, num_threads_, preallocated_normals_colored_msgs_);

            // ROS_DEBUG_STREAM_NAMED("timing","Generating can normals colored took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Publishing floor image");
            floor_im_pub_.publish(*image_ptr);
            // ROS_DEBUG("publish egocylindrical labels");
            // labels_im_pub_.publish(*labels_ptr);
            // ROS_DEBUG("publish egocylindrical labels colored");
            // labels_colored_im_pub_.publish(*labels_colored_ptr);
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "publish egocylindrical normals");
            normals_im_pub_.publish(*normals_ptr);
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "publish egocylindrical normals colored");
            normals_colored_im_pub_.publish(*normals_colored_ptr);

            // start = ros::WallTime::now();
            preallocated_can_msg_= std::make_shared<sensor_msgs::msg::Image>();
            preallocated_can_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
            // ROS_DEBUG_STREAM_NAMED("timing","Preallocating can image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

            // start = ros::WallTime::now();
            // preallocated_labels_msg_= std::make_shared<sensor_msgs::msg::Image>();
            // preallocated_labels_msg_->data.resize(labels_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
            // ROS_DEBUG_STREAM_NAMED("timing","Preallocating can labels took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
            // start = ros::WallTime::now();
            // preallocated_labels_colored_msg_= std::make_shared<sensor_msgs::msg::Image>();
            // preallocated_labels_colored_msg_->data.resize(labels_colored_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
            // ROS_DEBUG_STREAM_NAMED("timing","Preallocating can labels colored took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
            // start = ros::WallTime::now();
            preallocated_normals_msg_= std::make_shared<sensor_msgs::msg::Image>();
            preallocated_normals_msg_->data.resize(normals_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
            // ROS_DEBUG_STREAM_NAMED("timing","Preallocating can normals took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
            // start = ros::WallTime::now();
            preallocated_normals_colored_msgs_= std::make_shared<sensor_msgs::msg::Image>();
            preallocated_normals_colored_msgs_->data.resize(normals_colored_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
            // ROS_DEBUG_STREAM_NAMED("timing","Preallocating can normals colored took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        }

        totalEnd = std::chrono::steady_clock::now();
        totalTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(totalEnd - totalBegin).count();
        numberOfTotalCalls++;

        // log();
    }
}

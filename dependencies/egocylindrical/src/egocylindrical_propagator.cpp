//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>

namespace egocylindrical
{

    void EgoCylindricalPropagator::update(utils::SensorMeasurement& measurement)
    {
        totalBegin = std::chrono::steady_clock::now();

        updateBegin = std::chrono::steady_clock::now();

        // RCLCPP_INFO_STREAM(node_->get_logger(), "[EgoCylindricalPropagator::update()]");

        //TODO: Make ecwrapper pointers into local variables
        old_pts_ = wrapper_buffer_.getOld();
        
        // //TODO: Reset between runs so that the reset function returns once reset is complete
        // {
        //     WriteLock lock(reset_mutex_);
        //     if(should_reset_)
        //     {
        //         old_pts_ = nullptr;
        //         should_reset_ = false;
        //     }
        // }
        
        auto measurement_header = measurement.header;
        auto new_stamp = measurement_header.stamp;
        if (old_pts_)
        {
            rclcpp::Time old_stamp_ros = old_pts_->getHeader().stamp;
            rclcpp::Time new_stamp_ros = new_stamp;
            // ROS_INFO_STREAM_NAMED("timing", "old_pts_->getOneLabel(17772): " << std::hex << (uint16_t) old_pts_->getOneLabel(17772));

            if (old_stamp_ros > new_stamp_ros)
            {
                //old_pts_ = nullptr;
            }
            else if (old_stamp_ros == new_stamp_ros)
            {
              RCLCPP_WARN_STREAM(node_->get_logger(), "Repeat stamps received! " << new_stamp.sec << "." << new_stamp.nanosec << " seconds");
              //return;
            }
        }
        
                
        // if (old_pts_)
        // {
        //     // ROS_DEBUG_STREAM_NAMED("msg_timestamps","Previous stamp " << old_pts_->getHeader().stamp);
        // }

        //TODO: Warn of out-of-order images
        //TODO: If clock jumps back in time, reset egocylinder
        // ROS_INFO_STREAM_NAMED("update", "Measurement source: " << measurement.name);
        // ROS_DEBUG_STREAM_NAMED("msg_timestamps","Current stamp: " << new_stamp);
        // ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[egocylinder] Received [" << new_stamp << "] at [" << ros::WallTime::now() << "]");

        // ros::WallTime// start = ros::WallTime::now();
        
        if (!cfh_->updateTransforms(measurement_header))
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to update transforms!");
            return;
        }

        std_msgs::msg::Header target_header = cfh_->getTargetHeader();
        
        {
            if (old_pts_)
            {
                if (measurement.raytrace)
                {
                    new_pts_ = wrapper_buffer_.getNew(); // adds nullptr
                    try
                    {
                        pp_->transform(*old_pts_, *new_pts_, target_header); // , config_.num_threads
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN_STREAM(node_->get_logger(), "Problem finding transform:\n" << ex.what());
                        return; //TODO: After a configurable timeout, reset
                    }
                }
                else
                {
                    new_pts_ = wrapper_buffer_.reuseOld();
                }
            }
            else
            {
                new_pts_ = wrapper_buffer_.getNew();
                new_pts_->setHeader(target_header);
            }
            wrapper_buffer_.releaseOld();
        }

        updateEnd = std::chrono::steady_clock::now();
        updateTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(updateEnd - updateBegin).count();
        numberOfUpdateCalls++;

        // ROS_INFO_STREAM_NAMED("timing", "new_pts_->getOneLabel(17772) (before insert): " << std::hex << (uint16_t) new_pts_->getOneLabel(17772));


        insertBegin = std::chrono::steady_clock::now();
        if (propagated_ec_pub_->get_subscription_count())
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "propagated_ec_pub_ has subscribers, propagating points");

            // ros::WallTimet1 = ros::WallTime::now();
            utils::ECWrapper ec_copy = *new_pts_;
            // ros::WallTimet2 = ros::WallTime::now();
            utils::ECMsgConstPtr msg = ec_copy.getEgoCylinderPointsMsg();
            // ros::WallTimet3 = ros::WallTime::now();
            propagated_ec_pub_->publish(*msg);
            // ros::WallTimet4 = ros::WallTime::now();
            // ROS_DEBUG_STREAM_NAMED("timing", "Time to copy propagated points: " <<  (t2 - t1).toSec() * 1e3 << "ms");
            // ROS_DEBUG_STREAM_NAMED("timing", "Time to get message: " <<  (t3 - t2).toSec() * 1e3 << "ms");
            // ROS_DEBUG_STREAM_NAMED("timing", "Time to publish: " <<  (t4 - t3).toSec() * 1e3 << "ms");
            // ROS_DEBUG_STREAM_NAMED("timing", "Total time to copy & publish propagated points: " <<  (t4 - t1).toSec() * 1e3 << "ms");
        }

        {
            {
                measurement.insert(*new_pts_);
            }

            // ROS_INFO_STREAM_NAMED("timing", "new_pts_->getOneLabel(17772) (after insert): " << std::hex << (uint16_t) new_pts_->getOneLabel(17772));

            if (measurement.publish_update)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing updated measurement");
                
                if (ec_pub_->get_subscription_count() > 0 && shouldPublish(new_pts_))
                {
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "ec_pub_ has subscribers, publishing points");
                    // TODO: if no one is subscribing, we can propagate the points in place next time (if that turns out to be faster)
                    utils::ECMsgConstPtr msg = new_pts_->getEgoCylinderPointsMsg();

                    ec_pub_->publish(*msg);
                    // ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[egocylinder] Sent [" << msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
                    published(new_pts_);
                }

                if (info_pub_->get_subscription_count() > 0)
                {
                    info_pub_->publish(*new_pts_->getEgoCylinderInfoMsg());
                }
            }
        }
        insertEnd = std::chrono::steady_clock::now();
        insertTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(insertEnd - insertBegin).count();
        numberOfInsertCalls++;


        wrapper_buffer_.update();
        
        totalEnd = std::chrono::steady_clock::now();
        totalTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(totalEnd - totalBegin).count();
        numberOfTotalCalls++;

        // log();

        // ROS_DEBUG_STREAM_NAMED("timing", "Total time: " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
    }
    
    void EgoCylindricalPropagator::connectCB()
    {
        // If no one is listening, we can propagate points in place
        if (ec_pub_->get_subscription_count() == 0)
        {
            
        }
    }
    
    void EgoCylindricalPropagator::reset()
    {
        // WriteLock lock(reset_mutex_);
        // should_reset_ = true;
        wrapper_buffer_.reset();
    }
    
    // void EgoCylindricalPropagator::configCB(const egocylindrical::PropagatorConfig &config, uint32_t level)
    // {
    //     WriteLock lock(config_mutex_);
     
    //     // ROS_INFO_STREAM("Updating propagator config: height=" << config.height << ", width=" << config.width << ", vfov=" << config.vfov << ", can_width=" << config.can_width
    //     << ", v_offset=" << config.v_offset << ", cyl_radius=" << config.cyl_radius);
    //     config_ = config;
    // }
    
    
    bool EgoCylindricalPropagator::init()
    {
        // reconfigure_server_->setCallback(std::bind(&EgoCylindricalPropagator::configCB, this, _1, _2));
        
        // Get topic names
        std::string points_topic="egocylindrical_points", 
                    filtered_pc_topic="filtered_points", 
                    egocylinder_info_topic="egocylinder_info",
                    propagated_points_topic="propagated_egocylindrical_points";
        fixed_frame_id_ = "odom";

        // pnh_.getParam("points_out", points_topic );
        // pnh_.getParam("filtered_points", filtered_pc_topic);
        // pnh_.getParam("fixed_frame_id", fixed_frame_id_);
        node_->get_parameter("points_out", points_topic);
        node_->get_parameter("filtered_points", filtered_pc_topic);
        node_->get_parameter("fixed_frame_id", fixed_frame_id_);

        // // ROS_INFO_STREAM_NAMED("update", "points_topic: " << points_topic);
        // // ROS_INFO_STREAM_NAMED("update", "filtered_pc_topic: " << filtered_pc_topic);
        // // ROS_INFO_STREAM_NAMED("update", "egocylinder_info_topic: " << egocylinder_info_topic);
        // // ROS_INFO_STREAM_NAMED("update", "propagated_points_topic: " << propagated_points_topic);
        // // ROS_INFO_STREAM_NAMED("update", "fixed_frame_id_: " << fixed_frame_id_);


        cfh_->init();
        wrapper_buffer_.init();

        
        // reset_sub_ = nh_.subscribe<std_msgs::Empty>("reset", 1, [this](const std_msgs::Empty::ConstPtr&) { wrapper_buffer_.reset(2); });
        reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>("reset", 1, [this](const std_msgs::msg::Empty::ConstSharedPtr&) { wrapper_buffer_.reset(2); });

        // Setup publishers
        // ros::SubscriberStatusCallback image_cb = std::bind(&EgoCylindricalPropagator::connectCB, this);        
        // ec_pub_ = nh_.advertise<egocylindrical_msgs::msg::EgoCylinderPoints>(points_topic, 1, image_cb, image_cb);
        ec_pub_ = node_->create_publisher<egocylindrical_msgs::msg::EgoCylinderPoints>(points_topic, 1);

        //ros::SubscriberStatusCallback pc_cb = std::bind(&EgoCylindricalPropagator::connectCB, this);
        // pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_pc_topic, 3);
        info_pub_ = node_->create_publisher<egocylindrical_msgs::msg::EgoCylinderPoints>(egocylinder_info_topic, 1);

        propagated_ec_pub_ = node_->create_publisher<egocylindrical_msgs::msg::EgoCylinderPoints>(propagated_points_topic, 3);

        auto seq_cb = [this](utils::SensorMeasurement::Ptr measurement)
        {
          this->update(*measurement);
        };

        sensors_->init(fixed_frame_id_, seq_cb);
        pp_->init(fixed_frame_id_);

        return true;
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(rclcpp::Node::SharedPtr node):
        node_(node),
        // buffer_(node->get_clock()), // Initialize the buffer with the node's clock
        // tf_listener_(buffer_),
        // cfh_(buffer_, node_),
        // sensors_(node_, buffer_),
        // pp_(buffer_),
        wrapper_buffer_(), // Initialize the wrapper buffer with default config
        should_reset_(false)
    {
        // node_ = node;
        buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(node_->get_node_base_interface(),
                                                                            node_->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        cfh_ = std::make_shared<utils::CoordinateFrameHelper>(buffer_, node_);
        sensors_ = std::make_shared<utils::SensorCollection>(node_, buffer_);

        pp_ = std::make_shared<utils::PointPropagator>(buffer_);

        // reconfigure_server_ = std::make_shared<ReconfigureServer>(node_);

        initStartTime = std::chrono::steady_clock::now();
    }

    EgoCylindricalPropagator::~EgoCylindricalPropagator() {}
    
    void EgoCylindricalPropagator::log()
    {
        std::ofstream logFile;
        logFile.open("/home/masselmeier3/Desktop/Research/quad_pips_experiments/timing/superpixels/egocan/timing_log_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(initStartTime.time_since_epoch()).count()) + ".csv", std::ios::out);

        float averageUpdateTime = updateTimeTaken * 1.0e-3 / static_cast<float>(numberOfUpdateCalls);
        float averageInsertTime = insertTimeTaken * 1.0e-3 / static_cast<float>(numberOfInsertCalls);
        float averageTotalTime = totalTimeTaken * 1.0e-3 / static_cast<float>(numberOfTotalCalls);

        logFile << "avg update time (ms), avg insert time (ms), avg total time (ms), number of calls" << std::endl;
        logFile << averageUpdateTime << ", " << averageInsertTime << ", " << averageTotalTime << ", " << numberOfTotalCalls << std::endl;
        logFile.close();        
    }
      


}

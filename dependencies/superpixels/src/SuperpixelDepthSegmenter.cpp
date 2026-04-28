#include <superpixels/SuperpixelDepthSegmenter.h>

// , const std::string & config_path
SuperpixelDepthSegmenter::SuperpixelDepthSegmenter(const rclcpp::Node::SharedPtr & node)
{
    srand(123456789); // seed random calls

    // nh_ = nh;
    node_ = node;

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    RCLCPP_INFO_STREAM(node_->get_logger(), "   params_:");

    // Floor image parameters
    params_.k_c_ = node_->get_parameter("k_c").as_int();
    params_.v_fov_ = node_->get_parameter("v_fov").as_double() * M_PI / 180.0; // Convert degrees to radians
    params_.v_offset_ = node_->get_parameter("v_offset").as_double();
    params_.h_ = (params_.v_fov_ / 2.0) - params_.v_offset_;
    RCLCPP_INFO_STREAM(node_->get_logger(), "       k_c_: " << params_.k_c_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       v_fov_: " << params_.v_fov_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       v_offset_: " << params_.v_offset_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       h_: " << params_.h_);

    // Dilation parameters
    params_.num_dilation_iterations_ = node_->get_parameter("num_dilation_iterations").as_int();
    params_.kernel_radius_ = node_->get_parameter("kernel_radius").as_int();
    RCLCPP_INFO_STREAM(node_->get_logger(), "       num_dilation_iterations_: " << params_.num_dilation_iterations_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       kernel_radius_: " << params_.kernel_radius_);

    // Superpixel algorithm parameters
    params_.num_iterations_ = node_->get_parameter("num_iterations").as_int();
    params_.num_superpixels_ = node_->get_parameter("num_superpixels").as_int();
    int num_pixels = params_.k_c_ * params_.k_c_;
    params_.step_ = sqrt(num_pixels / (double) params_.num_superpixels_); // superpixel grid interval
    params_.warm_start_ = node_->get_parameter("warm_start").as_bool();
    params_.constraint_ = node_->get_parameter("constraint").as_bool();
    params_.ransac_ = node_->get_parameter("ransac").as_bool();
    params_.snapping_ = node_->get_parameter("snapping").as_bool();
    RCLCPP_INFO_STREAM(node_->get_logger(), "       num_superpixels_: " << params_.num_superpixels_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       num_iterations_: " << params_.num_iterations_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       step_: " << params_.step_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       warm_start_: " << params_.warm_start_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       constraint_: " << params_.constraint_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       ransac_: " << params_.ransac_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       snapping_: " << params_.snapping_);

    // Superpixel distance parameters
    params_.w_normal_ = node_->get_parameter("w_normal").as_double();
    params_.w_plane_dist_ = node_->get_parameter("w_plane_dist").as_double();
    params_.w_world_dist_ = node_->get_parameter("w_world_dist").as_double();
    params_.w_compact_ = node_->get_parameter("w_compact").as_double();
    RCLCPP_INFO_STREAM(node_->get_logger(), "       w_normal_: " << params_.w_normal_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       w_plane_dist_: " << params_.w_plane_dist_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       w_world_dist_: " << params_.w_world_dist_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       w_compact_: " << params_.w_compact_);

    // RANSAC parameters
    params_.ransac_K = node_->get_parameter("ransac_K").as_int();
    params_.ransac_N = node_->get_parameter("ransac_N").as_int();
    params_.ransac_T = node_->get_parameter("ransac_T").as_double();
    RCLCPP_INFO_STREAM(node_->get_logger(), "       ransac_K: " << params_.ransac_K);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       ransac_N: " << params_.ransac_N);
    RCLCPP_INFO_STREAM(node_->get_logger(), "       ransac_T: " << params_.ransac_T);

    // Set up subscribers and publishers
    image_transport::ImageTransport it(node_);

    depth_img_topic_ =  "/floor_image";
    // std::string label_img_topic =  "/egocylinder/floor_labels";
    normal_img_topic_ = "/floor_normals";

    // nh_.getParam("depth_img_topic", depth_img_topic);
    depth_img_topic_ = node_->get_parameter("depth_image_topic").as_string();

    // // nh_.getParam("label_img_topic", label_img_topic);
    // label_img_topic = node_->get_parameter("label_image_topic").as_string();

    // nh_.getParam("normal_img_topic", normal_img_topic);
    normal_img_topic_ = node_->get_parameter("normal_image_topic").as_string();

    // rclcpp::QoS qos = rclcpp::QoS(10);

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 3; // TODO: Make this a parameter

    raw_depth_img_sub_.subscribe(node_.get(), depth_img_topic_, "raw", qos); // Not sure if this is right
    // raw_label_img_sub_.subscribe(it, label_img_topic, 3);
    raw_normal_img_sub_.subscribe(node_.get(), normal_img_topic_, "raw", qos);// Not sure if this is right

    msg_sync_ = std::make_shared<MsgSynchronizer>(raw_depth_img_sub_, raw_normal_img_sub_, 3); // raw_label_img_sub_,
    msg_sync_->registerCallback(std::bind(&SuperpixelDepthSegmenter::allImageCallback, this, std::placeholders::_1, std::placeholders::_2)); // , _3

    prop_depth_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
    // prop_label_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
    prop_normal_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

    visited_ = cv::Mat(params_.k_c_, params_.k_c_, CV_8UC1, cv::Scalar(0));

    imagePreprocessor_ = new ImagePreprocessor(params_, node_);
    visualizer_ = new Visualizer(params_, node_);
    ransac_ = new Ransac(params_);
    convexHullifier_ = new ConvexHullifier(params_);

    callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&SuperpixelDepthSegmenter::parametersCallback, this, std::placeholders::_1));    

    initTime = std::chrono::steady_clock::now();
}

void SuperpixelDepthSegmenter::log()
{
    std::ofstream logFile;
    logFile.open("/home/masselmeier3/Desktop/Research/quad_pips_experiments/timing/superpixels/superpixels/timing_log_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(initTime.time_since_epoch()).count()) + ".csv", std::ios::out);

    float averageCleanTime = cleanTimeTaken * 1.0e-3 / static_cast<float>(numberOfCleanCalls);
    float averageFillTime = fillTimeTaken * 1.0e-3 / static_cast<float>(numberOfFillCalls);
    float averagePreprocessTime = preprocessTimeTaken * 1.0e-3 / static_cast<float>(numberOfPreprocessCalls);
    float averageSuperpixelTime = superpixelTimeTaken * 1.0e-3 / static_cast<float>(numberOfSuperpixelCalls);
    float averageRansacTime = ransacTimeTaken * 1.0e-3 / static_cast<float>(numberOfRansacCalls);
    float averageConvexHullTime = convexHullTimeTaken * 1.0e-3 / static_cast<float>(numberOfConvexHullCalls);
    float averageTotalTime = totalTimeTaken * 1.0e-3 / static_cast<float>(numberOfTotalCalls);

    logFile << "avg clean time (ms), avg fill time (ms), avg preprocess time (ms), avg superpixel time (ms), avg ransac time (ms), avg convex hull time (ms), avg total time (ms), number of calls" << std::endl;
    logFile << averageCleanTime << ", " << averageFillTime << ", " << averagePreprocessTime << ", " << averageSuperpixelTime << ", " << averageRansacTime << ", " << averageConvexHullTime << ", " << averageTotalTime << ", " << numberOfTotalCalls << std::endl;
    logFile.close();      
}

SuperpixelDepthSegmenter::~SuperpixelDepthSegmenter()
{  
    // delete tfListener_;
    delete imagePreprocessor_;
    delete visualizer_;
    delete ransac_;
    delete convexHullifier_;
}

rcl_interfaces::msg::SetParametersResult SuperpixelDepthSegmenter::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param: parameters)
    {
        // Floor image parameters
        if (param.get_name() == "k_c")
        {
            params_.k_c_ = param.get_value<int>();
        } else if (param.get_name() == "v_fov")
        {
            params_.v_fov_ = param.get_value<double>() * M_PI / 180.0; // Convert degrees to radians (just for reading in string)
        } else if (param.get_name() == "v_offset")
        {
            params_.v_offset_ = param.get_value<double>();
        }
        params_.h_ = (params_.v_fov_ / 2.0) - params_.v_offset_;
        
        // Dilation parameters
        if (param.get_name() == "num_dilation_iterations")
        {
            params_.num_dilation_iterations_ = param.get_value<int>();
        } else if (param.get_name() == "kernel_radius")
        {
            params_.kernel_radius_ = param.get_value<int>();
        }
        
        // Superpixel algorithm parameters
        if (param.get_name() == "num_iterations")
        {
            params_.num_iterations_ = param.get_value<int>();
        } else if (param.get_name() == "num_superpixels")
        {
            params_.num_superpixels_ = param.get_value<int>();
            int num_pixels = params_.k_c_ * params_.k_c_;
            params_.step_ = sqrt(num_pixels / (double) params_.num_superpixels_); // superpixel grid interval
        } else if (param.get_name() == "warm_start")
        {
            params_.warm_start_ = param.get_value<bool>();
        } else if (param.get_name() == "constraint")
        {
            params_.constraint_ = param.get_value<bool>();
        } else if (param.get_name() == "ransac")
        {
            params_.ransac_ = param.get_value<bool>();
        } else if (param.get_name() == "snapping")
        {
            params_.snapping_ = param.get_value<bool>();
        }
        
        // Superpixel distance parameters
        if (param.get_name() == "w_normal")
        {
            params_.w_normal_ = param.get_value<double>();
        } else if (param.get_name() == "w_plane_dist")
        {
            params_.w_plane_dist_ = param.get_value<double>();
        } else if (param.get_name() == "w_world_dist")
        {
            params_.w_world_dist_ = param.get_value<double>();
        } else if (param.get_name() == "w_compact")
        {
            params_.w_compact_ = param.get_value<double>();
        }
        
        // RANSAC parameters
        if (param.get_name() == "ransac_K")
        {
            params_.ransac_K = param.get_value<size_t>();
        } else if (param.get_name() == "ransac_N")
        {
            params_.ransac_N = param.get_value<int>();
        } else if (param.get_name() == "ransac_T")
        {
            params_.ransac_T = param.get_value<double>();
        } 
        // else
        // {
            // RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown parameter: " << param.get_name());
            // result.successful = false;
            // result.reason = "Unknown parameter";
        // }
    }

    convexHullifier_->setParams(params_);
    imagePreprocessor_->setParams(params_);
    visualizer_->setParams(params_);
    ransac_->setParams(params_);    
    return result;
}

// void SuperpixelDepthSegmenter::reconfigureCallback(superpixels::ParametersConfig &config, uint32_t level) 
// {
//     // Dilation parameters
//     params_.num_dilation_iterations_ = config.num_dilation_iterations;
//     params_.kernel_radius_ = config.kernel_radius;

//     // Superpixel algorithm parameters
//     params_.num_iterations_ = config.num_iterations;
//     params_.num_superpixels_ = config.num_superpixels;
//     int num_pixels = params_.k_c_ * params_.k_c_;
//     params_.step_ = sqrt(num_pixels / (double) params_.num_superpixels_); // superpixel grid interval
//     params_.constraint_ = config.constraint;
//     params_.ransac_ = config.ransac;
//     params_.snapping_ = config.snapping;

//     // Superpixel distance parameters
//     params_.w_normal_ = config.w_normal;
//     params_.w_plane_dist_ = config.w_plane_dist;
//     params_.w_compact_ = config.w_compact;

//     // RANSAC parameters
//     params_.ransac_K = config.ransac_K;
//     params_.ransac_N = config.ransac_N;
//     params_.ransac_T = config.ransac_T;

//     convexHullifier_->setParams(params_);
//     imagePreprocessor_->setParams(params_);
//     visualizer_->setParams(params_);
//     ransac_->setParams(params_);
// }

void SuperpixelDepthSegmenter::allImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg, 
                                                // const sensor_msgs::ImageConstPtr& label_image_msg, 
                                                const sensor_msgs::msg::Image::ConstSharedPtr& normal_image_msg)
{   
    std::lock_guard<std::mutex> lock(img_mutex_);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "[SuperpixelDepthSegmenter::allImageCallback]");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "       time stamp: " << depth_image_msg->header.stamp.sec << "." << depth_image_msg->header.stamp.nanosec);

    raw_depth_img_msg_ = depth_image_msg;
    // raw_label_img_msg_ = label_image_msg;
    raw_normal_img_msg_ = normal_image_msg;

    return;
}

bool SuperpixelDepthSegmenter::notReceivedDepthImage()
{
    return raw_depth_img_msg_ == nullptr;
}

// bool SuperpixelDepthSegmenter::notReceivedLabelImage()
// {
//     return (raw_label_img_msg_ == nullptr);
// }

bool SuperpixelDepthSegmenter::notReceivedNormalImage()
{
    return raw_normal_img_msg_ == nullptr;
}

bool SuperpixelDepthSegmenter::notReceivedImage()
{
    bool notReceivedDepth = notReceivedDepthImage();
    // bool notReceivedLabel = notReceivedLabelImage();
    bool notReceivedNormal = notReceivedNormalImage();

    if (notReceivedDepth)
        RCLCPP_WARN_STREAM(node_->get_logger(), "Not received depth image on topic: " << depth_img_topic_);

    // if (notReceivedLabel)
        // RCLCPP_WARN_STREAM(node_->get_logger(), "Not received label image.");

    if (notReceivedNormal)
        RCLCPP_WARN_STREAM(node_->get_logger(), "Not received normal image on topic: " << normal_img_topic_);

    return (notReceivedDepth || notReceivedNormal); // notReceivedLabel || 
}

void SuperpixelDepthSegmenter::run()
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[SuperpixelDepthSegmenter::run]");

    totalBegin = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(img_mutex_);

    if (notReceivedImage())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Not ready to segment, no images received yet.");
        return;
    } 
    // else
    // {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Received all images, proceeding with segmentation.");
    // }
 
    rclcpp::Time lookupTime = raw_depth_img_msg_->header.stamp;
    std::string egocan_frame = raw_depth_img_msg_->header.frame_id;

    // try
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "Looking up transform from " << egocan_frame << " to odom at time " << lookupTime.seconds() << "." << lookupTime.nanoseconds());
    //     tfBuffer_->lookupTransform("odom", egocan_frame, lookupTime); // Wait for transform
    // } catch (tf2::TransformException &ex)
    // {
    //     RCLCPP_ERROR_STREAM(node_->get_logger(), "Transform error: " << ex.what());
    //     return;
    // }

    // rclcpp::Duration timeout(3, 0); // 3 seconds
    bool canTransform = tfBuffer_->canTransform("odom", egocan_frame, lookupTime);    

    if (!canTransform)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Cannot transform from " << egocan_frame << " to odom at time " << lookupTime.seconds());
        return;
    }

    geometry_msgs::msg::TransformStamped egocanFrameToOdomFrame;
    try
    {
        egocanFrameToOdomFrame = tfBuffer_->lookupTransform("odom", egocan_frame, lookupTime); // , timeout
    }
    catch (tf2::TransformException & ex)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "   [Visualizer::publishPlanarRegions] TF lookup failed: " << ex.what());
        return;
    }    

    try
    {
        raw_depth_img_ptr_ = cv_bridge::toCvCopy(raw_depth_img_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
        // raw_label_img_ptr_ = cv_bridge::toCvCopy(raw_label_img_msg_, sensor_msgs::image_encodings::TYPE_8UC1);
        raw_normal_img_ptr_ = cv_bridge::toCvCopy(raw_normal_img_msg_, sensor_msgs::image_encodings::TYPE_32FC3);

    } catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // raw_depth_img_ptr_->image.size() != raw_label_img_ptr_->image.size() || 
    //  || raw_label_img_ptr_->image.size() != raw_normal_img_ptr_->image.size()
    if (raw_depth_img_ptr_->image.size() != raw_normal_img_ptr_->image.size()) 
    {
        RCLCPP_ERROR(node_->get_logger(), "Image sizes do not match.");
        return;
    }

    cleanBegin = std::chrono::steady_clock::now();

    cv::Mat raw_depth_img = raw_depth_img_ptr_->image;
    // cv::Mat raw_label_img = raw_label_img_ptr_->image;
    cv::Mat raw_normal_img = raw_normal_img_ptr_->image;

    // Pre-processing
    // calculateStep(raw_depth_img);

    // Health check
    // healthCheck(raw_depth_img, raw_label_img, raw_normal_img);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning images ...");

    // Clean images
    cv::Mat cleaned_depth_img, cleaned_normal_img; // cleaned_label_img, 
    imagePreprocessor_->cleanImages(raw_depth_img, raw_normal_img, // raw_label_img,  
                                    cleaned_depth_img, cleaned_normal_img, visited_); // cleaned_label_img, 

    // RCLCPP_INFO_STREAM(node_->get_logger(), " after cleaning, (480, 480) is: " << cleaned_depth_img.at<float>(480, 480));

    cleanEnd = std::chrono::steady_clock::now();
    cleanTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(cleanEnd - cleanBegin).count();
    numberOfCleanCalls++;

    // int64_t clean_total_time = std::chrono::duration_cast<std::chrono::microseconds>(cleanEnd - cleanBegin).count();
    // double clean_total_time_sec = 1.0e-6 * clean_total_time; 

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Filling images ...");

    fillBegin = std::chrono::steady_clock::now();

    double default_height = egocanFrameToOdomFrame.transform.translation.z; //   - 0.05
    // double default_height = 0.575;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "   Default height for filling: " << default_height);

    cv::Mat filled_depth_img, filled_normal_img; // filled_label_img, 
    imagePreprocessor_->fillInImage(cleaned_depth_img, cleaned_normal_img, visited_, // cleaned_label_img, 
                                    filled_depth_img, filled_normal_img, default_height); // , raw_depth_img_ptr_, filled_label_img, 

    // RCLCPP_INFO_STREAM(node_->get_logger(), " after filling, (480, 480) is: " << filled_depth_img.at<float>(480, 480));

    fillEnd = std::chrono::steady_clock::now();
    fillTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(fillEnd - fillBegin).count();
    numberOfFillCalls++;

    // int64_t fill_total_time = std::chrono::duration_cast<std::chrono::microseconds>(fillEnd - fillBegin).count();
    // double fill_total_time_sec = 1.0e-6 * fill_total_time;

    // Health check
    // healthCheck(cleaned_depth_img, cleaned_label_img, cleaned_normal_img);

    preprocessBegin = std::chrono::steady_clock::now();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Preprocessing images ...");

    // Pre-processing
    imagePreprocessor_->preprocessImages(filled_depth_img, filled_normal_img, // filled_label_img, 
                                            preprocessed_depth_img, preprocessed_normal_img); // preprocessed_label_img, 

    // RCLCPP_INFO_STREAM(node_->get_logger(), " after preprocessing, (480, 480) is: " << preprocessed_depth_img.at<float>(480, 480));

    preprocessEnd = std::chrono::steady_clock::now();
    // int64_t preprocess_total_time = std::chrono::duration_cast<std::chrono::microseconds>(preprocessEnd - preprocessBegin).count();
    // double preprocess_total_time_sec = 1.0e-6 * preprocess_total_time;
    preprocessTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(preprocessEnd - preprocessBegin).count();
    numberOfPreprocessCalls++;

    // Health check
    // healthCheck(preprocessed_depth_img, preprocessed_label_img, preprocessed_normal_img);

    superpixelBegin = std::chrono::steady_clock::now();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Clearing ...");

    // Clear data
    reset_data(preprocessed_depth_img, preprocessed_normal_img); // preprocessed_label_img, 

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Generating superpixels ...");

    // Generate superpixels
    generateSuperpixels(preprocessed_depth_img, preprocessed_normal_img); // preprocessed_label_img, 

    superpixelEnd = std::chrono::steady_clock::now();
    superpixelTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(superpixelEnd - superpixelBegin).count();
    numberOfSuperpixelCalls++;
    // int64_t superpixel_total_time = std::chrono::duration_cast<std::chrono::microseconds>(superpixelEnd - superpixelBegin).count();
    // double superpixel_total_time_sec = 1.0e-6 * superpixel_total_time;

    // Calculate convex hulls
    convexHullBegin = std::chrono::steady_clock::now();
    convexHullifier_->run(centers_, center_counts_, superpixels_, superpixel_projections_, 
                            superpixel_convex_hulls_, egocan_to_region_rotations_, preprocessed_depth_img);
    convexHullEnd = std::chrono::steady_clock::now();
    convexHullTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(convexHullEnd - convexHullBegin).count();
    numberOfConvexHullCalls++;
    // int64_t convex_hull_total_time = std::chrono::duration_cast<std::chrono::microseconds>(convexHullEnd - convexHullBegin).count();
    // double convex_hull_total_time_sec = 1.0e-6 * convex_hull_total_time;

    // RCLCPP_INFO_STREAM(node_->get_logger(),  "Timing ---- \n" << 
    //                             "   Image cleaning took: " << clean_total_time_sec << " seconds, \n" <<
    //                             "   Image filling took: " << fill_total_time_sec << " seconds, \n" <<
    //                             "   Image preprocessing took " << preprocess_total_time_sec << " seconds, \n" <<
    //                             "   Superpixels took " << superpixel_total_time_sec << " seconds, \n" << 
    //                             // "   Region splitting took " << region_split_total_time_sec << " seconds, \n" <<
    //                             "   Convex hulls took " << convex_hull_total_time_sec << " seconds, \n" <<
    //                             "   Number of superpixels: " << centers_.size() << "\n" <<
    //                             "   Total: " << clean_total_time_sec + fill_total_time_sec + preprocess_total_time_sec + superpixel_total_time_sec + convex_hull_total_time_sec << " seconds");


    totalEnd = std::chrono::steady_clock::now();
    totalTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(totalEnd - totalBegin).count();
    numberOfTotalCalls++;
    // RCLCPP_INFO_STREAM(node_->get_logger(),  "Loop done");

    return;
}

void SuperpixelDepthSegmenter::visualize()
{
    // RCLCPP_INFO_STREAM(node_->get_logger(),  "Visualization start");

    if (preprocessed_depth_img.empty() || preprocessed_normal_img.empty())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Cannot visualize, preprocessed images are empty.");
        return;
    }

    if (raw_depth_img_ptr_ == nullptr || raw_normal_img_ptr_ == nullptr)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Cannot visualize, raw image pointers are null.");
        return;
    }

    if (centers_.empty() || clusters_.empty() || center_counts_.empty())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Cannot visualize, superpixel data is empty.");
        return;
    }

    if (superpixel_projections_.empty() || superpixel_convex_hulls_.empty() || egocan_to_region_rotations_.empty())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Cannot visualize, superpixel region data is empty.");
        return;
    }

    // Visualize
    // visBegin = std::chrono::steady_clock::now();
    visualizer_->visualize(preprocessed_depth_img, 
                            preprocessed_normal_img,
                            raw_depth_img_ptr_,
                            raw_normal_img_ptr_,
                            centers_,
                            clusters_,
                            center_counts_,
                            superpixel_projections_,
                            superpixel_convex_hulls_,
                            egocan_to_region_rotations_);
    // visEnd = std::chrono::steady_clock::now();
    // int64_t vis_total_time = std::chrono::duration_cast<std::chrono::microseconds>(visEnd - visBegin).count();
    // double vis_total_time_sec = 1.0e-6 * vis_total_time;
    // *node_->get_clock(), 3,
    // RCLCPP_INFO_STREAM(node_->get_logger(),  "Visualization done");

}


void SuperpixelDepthSegmenter::reset_data(const cv::Mat & depth_image,
                                            // const cv::Mat & label_image,
                                            const cv::Mat & normal_image)
{
    if (params_.warm_start_ && initialized_)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Resetting data ...");
        clusters_ = cv::Mat(depth_image.size(), CV_32S, cv::Scalar(-1)); // 32-bit signed integer
        distances_ = cv::Mat(depth_image.size(), CV_64F, cv::Scalar(std::numeric_limits<double>::max())); // 64-bit floating-point

        // Keep centers as is

        center_counts_.assign(center_counts_.size(), 0);
    } else
    {
        // Cold-starting, or initializing for the first time
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Clearing data ...");
        
        // RCLCPP_INFO_STREAM(node_->get_logger(), "   first mats ...");
        // cv mats
        clusters_.release();
        distances_.release();

        // RCLCPP_INFO_STREAM(node_->get_logger(), "   clearing vectors ...");
        centers_.clear();
        center_counts_.clear();

        superpixels_.clear();
        superpixel_projections_.clear();
        superpixel_convex_hulls_.clear();
        egocan_to_region_rotations_.clear();

        // RCLCPP_INFO_STREAM(node_->get_logger(), "Initializing data ...");

        // Will populate clusters_, distances_, centers_, and center_counts_
        init_data(depth_image, normal_image); // label_image,

        initialized_ = true;
    }

}

void SuperpixelDepthSegmenter::init_data(const cv::Mat & depth_image,
                                        //  const cv::Mat & label_image,
                                         const cv::Mat & normal_image)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::init_data]");

    /* Initialize the cluster and distance matrices. */
    clusters_ = cv::Mat(depth_image.size(), CV_32S, cv::Scalar(-1)); // 32-bit signed integer
    distances_ = cv::Mat(depth_image.size(), CV_64F, cv::Scalar(std::numeric_limits<double>::max())); // 64-bit floating-point

    // RCLCPP_INFO_STREAM(node_->get_logger(), "    clusters_.size: " << clusters_.size() << ", type: " << clusters_.type() << ", channels: " << clusters_.channels());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    distances_.size: " << distances_.size() << ", type: " << distances_.type() << ", channels: " << distances_.channels());

    /* Initialize the centers and counters. */
    for (int r = params_.step_; r < depth_image.rows - (params_.step_ / 2); r += params_.step_)
    {
        for (int c = params_.step_; c < depth_image.cols - (params_.step_ / 2); c += params_.step_)
        {        
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       (r, c): (" << r << ", " << c << ")");

            // float depth = depth_image.at<float>(r, c);

            // if (std::isnan(depth) || std::fabs(depth) < 1e-6)
            // {
            //     continue;
            // }

            std::vector<double> center;

            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Finding local minimum ...");
            /* Find the local minimum (gradient-wise). */
            cv::Point originalCenter(c, r);
            // cv::Point localMinimum = findLocalMinimum(depth_image, normal_image, originalCenter); // label_image, 
            cv::Point localMinimum = findCentroid(depth_image, normal_image, originalCenter); // label_image,

            if (!isPixelValid(depth_image, normal_image, localMinimum, params_.k_c_)) // label_image, 
            {
            //     RCLCPP_INFO_STREAM(node_->get_logger(), "       Invalid local minimum found");
            //     RCLCPP_INFO_STREAM(node_->get_logger(), "           setting (" << r << ", " << c << ") to default ground floor value ...");
            //     // augment center to be default ground floor value
            //     localMinimum = originalCenter;
            //     depth = 0.385;
            //     label = 3;
            //     normal = cv::Vec3f(0.0, -1.0, 0.0);                
                continue;
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(), "       local minimum found at (" << localMinimum.x << ", " << localMinimum.y << ")");

            float depth = depth_image.at<float>(localMinimum.y, localMinimum.x);
            // uint8_t label = label_image.at<uint8_t>(localMinimum.y, localMinimum.x);
            cv::Vec3f normal = normal_image.at<cv::Vec3f>(localMinimum.y, localMinimum.x);

            // RCLCPP_INFO_STREAM(node_->get_logger(), "       depth: " << depth);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       label: " << (int) label);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       normal: (" << normal.val[0] << ", " << normal.val[1] << ", " << normal.val[2] << ")");

            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Valid local minimum found, pushing back");
            /* Generate the center vector. */
            center.push_back(localMinimum.x);
            center.push_back(localMinimum.y);
            center.push_back(depth);
            // center.push_back(label);
            center.push_back(normal.val[0]);
            center.push_back(normal.val[1]);
            center.push_back(normal.val[2]);

            /* Append to vector of centers. */
            centers_.push_back(center);
            center_counts_.push_back(0);
        }
    }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       centers_.size(): " << centers_.size());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "       center_counts_.size(): " << center_counts_.size());

}

cv::Point SuperpixelDepthSegmenter::findCentroid(const cv::Mat & depth_image, 
                                                        // const cv::Mat & label_image,
                                                        const cv::Mat & normal_image,
                                                        const cv::Point & og_center)
{
    cv::Point centroid(0, 0);
    int count = 0;

    int delta = params_.step_ / 4; // 5;

    for (int d = 0; d < delta; d++)
    {
        for (int r = og_center.y - delta; r <= og_center.y + delta; r++)
        {
            for (int c = og_center.x - delta; c <= og_center.x + delta; c++)
            {
                cv::Point current(c, r);

                if (!isPixelValid(depth_image, normal_image, current, params_.k_c_)) // label_image, 
                {
                    continue;
                } else
                {
                    centroid.x += c;
                    centroid.y += r;
                    count++;
                }
            }
        }

        if (count > 0)
        {
            centroid.x /= count;
            centroid.y /= count;
        } else
        {
            return cv::Point(-1, -1);
        }
    }

    // find the valid pixel closest to the centroid
    cv::Point loc_min(-1, -1);
    double min_dist = std::numeric_limits<double>::max();
    for (int d = 0; d < delta; d++)
    {
        for (int r = og_center.y - delta; r <= og_center.y + delta; r++)
        {
            for (int c = og_center.x - delta; c <= og_center.x + delta; c++)
            {
                cv::Point current(c, r);

                if (!isPixelValid(depth_image, normal_image, current, params_.k_c_)) // label_image, 
                {
                    continue;
                } else
                {
                    double dist = sqrt(pow(current.x - centroid.x, 2) + pow(current.y - centroid.y, 2));

                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        loc_min = current;
                    }
                }
            }
        }

        if (isPixelInBounds(params_.k_c_, loc_min))
        {
            break;
        }
    }

    return loc_min;
}

cv::Point SuperpixelDepthSegmenter::findLocalMinimum(const cv::Mat & depth_image, 
                                                        // const cv::Mat & label_image,
                                                        const cv::Mat & normal_image,
                                                        const cv::Point & og_center)
{
    // double min_grad = std::numeric_limits<double>::max();
    cv::Point loc_min(-1, -1);
    // const cv::Point og_center = loc_min; 

    int delta = params_.step_ / 4; // 5;

    for (int d = 0; d < delta; d++)
    {
        for (int r = og_center.y - delta; r <= og_center.y + delta; r++)
        {
            for (int c = og_center.x - delta; c <= og_center.x + delta; c++)
            {
                cv::Point current(c, r);

                // if (!isPixelInBounds(depth_image, current))
                // {
                //     continue;
                // }

                // double grad = sqrt(pow(color.val[0] - center_color.val[0], 2) +
                //                    pow(color.val[1] - center_color.val[1], 2) +
                //                    pow(color.val[2] - center_color.val[2], 2));

                if (!isPixelValid(depth_image, normal_image, current, params_.k_c_)) // label_image, 
                {
                    continue;
                } else
                {
                    float depth = depth_image.at<float>(current.y, current.x);

                    if (isPixelInBounds(params_.k_c_, loc_min)) // have found a valid pixel in the region, can compare now
                    {
                        if (depth < depth_image.at<float>(loc_min.y, loc_min.x))
                        {
                            loc_min = cv::Point(c, r);
                        }
                    } else // have not found a valid pixel in the region yet
                    {
                        loc_min = cv::Point(c, r);
                    }
                }
            }
        }

        if (isPixelInBounds(params_.k_c_, loc_min))
        {
            break;
        }
    }

    return loc_min;
}

void SuperpixelDepthSegmenter::generateSuperpixels(const cv::Mat & depth_image,
                                                    // const cv::Mat & label_image,
                                                    const cv::Mat & normal_image)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::generateSuperpixels]");

    srand(1);

    cv::Point current, center, new_center;

    float depth;
    // uint8_t label;
    cv::Vec3f normal;
    cv::Vec3f candidate_normal;

    bool check;
    double dist;

    int cluster_id;

    // Generate superpixels
    for (int i = 0; i < params_.num_iterations_; i++)
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Iteration: " << i);

        /* Reset distance and cluster values. */
        distances_ = cv::Mat(depth_image.size(), CV_64F, cv::Scalar(std::numeric_limits<double>::max()));
        // clusters_ = cv::Mat(depth_image.size(), CV_32S, cv::Scalar(-1)); // 32-bit signed integer

        /* Update distances and clusters */
        for (size_t j = 0; j < centers_.size(); j++) 
        {
            /* Only compare to pixels in a 2 x step by 2 x step region. */
            for (int r = centers_[j][1] - params_.step_; r < centers_[j][1] + params_.step_; r++) 
            {
                // #pragma omp parallel for
                for (int c = centers_[j][0] - params_.step_; c < centers_[j][0] + params_.step_; c++) 
                {                
                    current = cv::Point(c, r);
                    if (isPixelValid(depth_image, normal_image, current, params_.k_c_)) 
                    {
                        depth = depth_image.at<float>(r, c);
                        normal = normal_image.at<cv::Vec3f>(r, c);

                        check = params_.constraint_ ? checkConstraints(j, depth, normal, current) : true;

                        dist = computeDistance(j, 
                                                depth,
                                                normal,
                                                current);

                        if (check && dist < distances_.at<double>(r, c)) 
                        {
                            distances_.at<double>(r, c) = dist;
                            clusters_.at<int>(r, c) = j;
                        }
                    }
                }
            }
        }

        /* Clear the center values. */
        for (size_t j = 0; j < centers_.size(); j++) 
        {
            centers_[j][0] = 0; // x
            centers_[j][1] = 0; // y
            centers_[j][2] = 0; // depth
            centers_[j][3] = 0; // normal x
            centers_[j][4] = 0; // normal y
            centers_[j][5] = 0; // normal z
            center_counts_[j] = 0;
        }

        superpixels_ = std::vector<std::vector<cv::Point>>(centers_.size());
        /* Compute the new cluster centers. */
        for (int r = 0; r < depth_image.rows; r++) 
        {
            for (int c = 0; c < depth_image.cols; c++) 
            {            
                current = cv::Point(c, r);
                
                cluster_id = clusters_.at<int>(r, c);

                if (cluster_id != -1) 
                {
                    float depth = depth_image.at<float>(r, c);
                    cv::Vec3f normal = normal_image.at<cv::Vec3f>(r, c);

                    centers_[cluster_id][0] += c;
                    centers_[cluster_id][1] += r;
                    centers_[cluster_id][2] += depth;
                    centers_[cluster_id][3] += normal.val[0];
                    centers_[cluster_id][4] += normal.val[1];
                    centers_[cluster_id][5] += normal.val[2];
                    
                    center_counts_[cluster_id] += 1; 

                    superpixels_[cluster_id].push_back(current);
                }
            }
        }     

        /* Normalize the clusters. */
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Normalizing clusters ...");
        for (size_t j = 0; j < centers_.size(); j++) 
        {
            if (center_counts_[j] == 0) 
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "           Center " << j << " is empty.");
                continue;
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(), "           Center " << j << ", count: " << center_counts_[j]);

            // Average
            centers_[j][0] /= center_counts_[j];
            centers_[j][1] /= center_counts_[j];
            centers_[j][2] /= center_counts_[j];
            centers_[j][3] /= center_counts_[j];
            centers_[j][4] /= center_counts_[j];
            centers_[j][5] /= center_counts_[j];

            // Round pixel
            centers_[j][0] = int(round(centers_[j][0]));
            centers_[j][1] = int(round(centers_[j][1]));

            // Re-normalize
            cv::Vec3f normal = cv::Vec3f(centers_[j][3], centers_[j][4], centers_[j][5]);
            cv::Vec3f re_normal = normal / cv::norm(normal);
            centers_[j][3] = re_normal.val[0];
            centers_[j][4] = re_normal.val[1];
            centers_[j][5] = re_normal.val[2];
        }

        /* Snap clusters to nearest pixel */
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Refining via RANSAC ...");
        for (size_t j = 0; j < centers_.size(); j++) 
        {
            if (center_counts_[j] == 0) 
            {
                continue;
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(), "       [" << j << "]: ");

            // RCLCPP_INFO_STREAM(node_->get_logger(), "           superpixels size: " << superpixels_[j].size());

            if (params_.snapping_)
            {
                center = cv::Point(centers_[j][0], centers_[j][1]);
                new_center = findClosestPixel(j, center, depth_image, normal_image); // label_image, 
                depth = depth_image.at<float>(new_center.y, new_center.x);
                normal = normal_image.at<cv::Vec3f>(new_center.y, new_center.x);
                centers_[j][0] = new_center.x;
                centers_[j][1] = new_center.y;
                centers_[j][2] = depth;
                centers_[j][3] = normal.val[0];
                centers_[j][4] = normal.val[1];
                centers_[j][5] = normal.val[2];
            }

            if (params_.ransac_)
            {
                // refine normal via RANSAC
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           pixel: " << centers_[j][0] << ", " << centers_[j][1]);
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           depth: " << centers_[j][2]);
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           normal: " << centers_[j][3] << ", " << centers_[j][4] << ", " << centers_[j][5]);
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           counts: " << center_counts_[j]);
                
                ransacBegin = std::chrono::steady_clock::now();

                cv::Vec3f normal = cv::Vec3f(centers_[j][3], centers_[j][4], centers_[j][5]);
                candidate_normal = ransac_->run(superpixels_[j], depth_image, normal);
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           post-ransac normal: " << candidate_normal.val[0] << ", " << candidate_normal.val[1] << ", " << candidate_normal.val[2]);

                centers_[j][3] = candidate_normal.val[0];
                centers_[j][4] = candidate_normal.val[1];
                centers_[j][5] = candidate_normal.val[2];

                ransacEnd = std::chrono::steady_clock::now();
                ransacTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(ransacEnd - ransacBegin).count();
                numberOfRansacCalls++;
            }
        }
    }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       centers:");
    // for (int j = 0; j < center_counts_.size(); j++)
    // {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "           [" << j << "]: ");
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               pixel: " << centers_[j][0] << ", " << centers_[j][1]);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               depth: " << centers_[j][2]);
    //     // RCLCPP_INFO_STREAM(node_->get_logger(), "               label: " << centers_[i][3]);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               normal: " << centers_[j][3] << ", " << centers_[j][4] << ", " << centers_[j][5]);
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "               counts: " << center_counts_[j]);
    // }
}

bool SuperpixelDepthSegmenter::checkConstraints(const int & center_idx, 
                                                const float & depth,
                                                // const uint8_t & label,
                                                const cv::Vec3f & normal,
                                                const cv::Point & pixel)
{
    cv::Point center_pixel = cv::Point(centers_[center_idx][0], centers_[center_idx][1]);
    float center_depth = centers_[center_idx][2];
    // uint8_t center_label = centers_[center_idx][3];
    cv::Vec3f center_normal = cv::Vec3f(centers_[center_idx][3], centers_[center_idx][4], centers_[center_idx][5]);

    cv::Vec3f egocanPt;
    pixelToEgocanFrame(egocanPt, pixel, depth, params_.k_c_, params_.h_);

    cv::Vec3f centerEgocanPt;
    pixelToEgocanFrame(centerEgocanPt, center_pixel, center_depth, params_.k_c_, params_.h_);

    bool normal_check = normal.dot(center_normal) > 0.75;

    bool plane_distance_check = std::abs( (centerEgocanPt - egocanPt).dot(center_normal))  < 0.01;

    return normal_check && plane_distance_check;
}

double SuperpixelDepthSegmenter::computeDistance(const int & center_idx, 
                                                    const float & depth,
                                                    // const uint8_t & label,
                                                    const cv::Vec3f & normal,
                                                    const cv::Point & pixel)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::computeDistance]");

    cv::Point center_pixel = cv::Point(centers_[center_idx][0], centers_[center_idx][1]);
    float center_depth = centers_[center_idx][2];
    // uint8_t center_label = centers_[center_idx][3];
    cv::Vec3f center_normal = cv::Vec3f(centers_[center_idx][3], centers_[center_idx][4], centers_[center_idx][5]);

    cv::Vec3f egocanPt;
    pixelToEgocanFrame(egocanPt, pixel, depth, params_.k_c_, params_.h_);

    cv::Vec3f centerEgocanPt;
    pixelToEgocanFrame(centerEgocanPt, center_pixel, center_depth, params_.k_c_, params_.h_);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_idx: " << center_idx);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_pixel: (r:" << center_pixel.y << ", c: " << center_pixel.x << ")");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_world_pt: " << centerEgocanPt);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_depth: " << center_depth);
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_label: " << center_label);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_normal: " << center_normal);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           pixel: (r: " << pixel.y << ", c: " << pixel.x << ")");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           world_pt: " << egocanPt);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           depth: " << depth);
    // // RCLCPP_INFO_STREAM(node_->get_logger(), "           label: " << label);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           normal: " << normal);


    // Normal term
    double d_normal = (1.0 - normal.dot(center_normal));

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           normal: " << normal);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_normal: " << center_normal);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           d_normal: " << d_normal);

    double max_d_normal = 2.0;
    double weighted_d_normal = params_.w_normal_ * (d_normal / max_d_normal);

    // if (d_normal > max_d_normal)
    // {
    //     RCLCPP_WARN_STREAM(node_->get_logger(), "       d_normal exceeds max, d_normal: " << d_normal << ", max_d_normal: " << max_d_normal);
    // }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           d_normal: " << d_normal);

    // Plane distance term

    double d_plane = std::abs( (egocanPt - centerEgocanPt).dot(center_normal) );

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           egocanPt: " << egocanPt);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           centerEgocanPt: " << centerEgocanPt);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           d_plane: " << d_plane);
    double max_d_plane = params_.v_fov_;
    double weighted_d_plane = params_.w_plane_dist_ * (d_plane / max_d_plane);

    // if (d_plane > max_d_plane)
    // {
    //     RCLCPP_WARN_STREAM(node_->get_logger(), "       d_plane exceeds max, d_plane: " << d_plane << ", max_d_plane: " << max_d_plane);
    // }

    double d_world = cv::norm(egocanPt - centerEgocanPt);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           egocanPt: " << egocanPt);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           centerEgocanPt: " << centerEgocanPt);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           d_world: " << d_world);
    double max_d_world = params_.v_fov_;
    double weighted_d_world = params_.w_world_dist_ * (d_world / max_d_world);

    // if (d_world > max_d_world)
    // {
    //     RCLCPP_WARN_STREAM(node_->get_logger(), "       d_world exceeds max, d_world: " << d_world << ", max_d_world: " << max_d_world);
    // }

    // Compactness term
    double d_compact = sqrt(pow(center_pixel.x - pixel.x, 2) + pow(center_pixel.y - pixel.y, 2));

    // RCLCPP_INFO_STREAM(node_->get_logger(), "           center_pixel: (r:" << center_pixel.y << ", c: " << center_pixel.x << ")");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           pixel: (r: " << pixel.y << ", c: " << pixel.x << ")");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "           d_compact: " << d_compact);

    double max_compact_dist = sqrt(pow(params_.step_, 2) + pow(params_.step_, 2));
    double weighted_d_compact = params_.w_compact_ * (d_compact / max_compact_dist);

    // if (d_compact > max_compact_dist)
    // {
    //     RCLCPP_WARN_STREAM(node_->get_logger(), "       d_compact exceeds max, d_compact: " << d_compact << ", max_compact_dist: " << max_compact_dist);
    // }

    return weighted_d_normal + weighted_d_plane + weighted_d_world + weighted_d_compact;
}

cv::Point SuperpixelDepthSegmenter::findClosestPixel(const int & center_idx,
                                                        const cv::Point & center, 
                                                        const cv::Mat & depth_image,
                                                        // const cv::Mat & label_image,
                                                        const cv::Mat & normal_image)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::findClosestPixel]");

    cv::Point new_center = center;

    // inefficient
    for (int delta = 1; delta < params_.step_; delta++)
    {
        for (int i = -delta; i <= delta; i++)
        {
            for (int j = -delta; j <= delta; j++)
            {
                cv::Point current(center.x + j, center.y + i);

                if (isPixelValid(depth_image, normal_image, current, params_.k_c_) && // label_image, 
                    clusters_.at<int>(current.y, current.x) == center_idx)
                {
                    new_center = current;
                    return new_center;
                }
            }
        }
    }

    return new_center;
}
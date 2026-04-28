#include <depth_img_normal_estimation/NormalEstimator.h>

float DELTA = std::numeric_limits<float>::epsilon();

NormalEstimator::NormalEstimator(const rclcpp::Node::SharedPtr & nodePtr, 
                                    const std::string & camera_depth_topic, 
                                    const std::string & config_path,
                                    const bool & hardware)
{
    nodePtr_ = nodePtr;

    image_transport::ImageTransport it(nodePtr_);

    // Initialize subscriber
    depth_img_sub = it.subscribe(camera_depth_topic, 1, &NormalEstimator::depthImgCallback, this);

    // Initialize publishers
    normals_pub = it.advertise("/camera/normals", 1);
    normals_bgr_img_pub = it.advertise("/camera/color_normals", 1);
    filtered_depth_pub = it.advertise("/camera/depth/filtered", 1);

    // Load configs
    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "        config_path: " << config_path);
    YAML::Node configYamlNode = YAML::LoadFile(config_path);

    params.depth_thresh = configYamlNode["normal_estimation"]["depth_threshold"].as<float>();
    params.bilat_filter_num_iters = configYamlNode["bilateral_filter"]["num_iters"].as<int>();
    params.bilat_filter_kernel_size = configYamlNode["bilateral_filter"]["kernel_size"].as<int>();
    params.bilat_filter_sigma_color = configYamlNode["bilateral_filter"]["sigma_color"].as<float>();
    params.bilat_filter_sigma_space = configYamlNode["bilateral_filter"]["sigma_space"].as<float>();
    params.infill_filter_kernel_size = configYamlNode["infill_filter"]["kernel_size"].as<int>();

    hardware_ = hardware;

    filtered_depth_ptr.reset(new cv_bridge::CvImage);
    normals_ptr.reset(new cv_bridge::CvImage);
    normals_bgr_ptr.reset(new cv_bridge::CvImage);

    // initTime = std::chrono::steady_clock::now();
}

// void NormalEstimator::log()
// {
//     // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), " [NormalEstimator::~NormalEstimator]");
//     std::ofstream logFile;
//     logFile.open("/home/masselmeier3/Desktop/Research/quad_pips_experiments/timing/superpixels/normals/timing_log_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(initTime.time_since_epoch()).count()) + ".csv", std::ios::out);

//     float averagePreprocessTime = preprocessTimeTaken * 1.0e-3 / static_cast<float>(numberOfPreprocessCalls);
//     float averagePaddingTime = paddingTimeTaken * 1.0e-3 / static_cast<float>(numberOfPaddingCalls);
//     float averageDepthGradientsTime = depthGradientsTimeTaken * 1.0e-3 / static_cast<float>(numberOfDepthGradientsCalls);
//     float averageTotalTime = totalTimeTaken * 1.0e-3 / static_cast<float>(numberOfTotalCalls);

//     logFile << "avg preprocess time (ms), avg padding time (ms), avg depth gradients time (ms), avg total time (ms), number of calls" << std::endl;
//     logFile << averagePreprocessTime << ", " << averagePaddingTime << ", " << averageDepthGradientsTime << ", " << averageTotalTime << ", " << numberOfTotalCalls << std::endl;
//     logFile.close();
// }

void NormalEstimator::depthImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    // std::lock_guard<std::mutex> lock(depth_img_mutex);

    // ROS_INFO("[NormalEstimator::depthImgCallback]");

    try
    {
        depth_img_ptr = cv_bridge::toCvCopy(msg, "32FC1");
        // ROS_INFO("      received new depth image");
    } catch (std::exception& e)
    {
        RCLCPP_ERROR_STREAM(nodePtr_->get_logger(), "       depthImgCallback failed: " << e.what());
        return;
    }       

    runNormalEstimation();

}

bool NormalEstimator::notReceivedDepthImage()
{
    return depth_img_ptr == nullptr;
}

void NormalEstimator::runNormalEstimation()
{
    // std::lock_guard<std::mutex> lock(depth_img_mutex);

    if (notReceivedDepthImage())
    {
        RCLCPP_WARN_STREAM(nodePtr_->get_logger(), "Not ready to estimate normals, no depth image received yet.");
        return;
    }

    // totalBegin = std::chrono::steady_clock::now();

    // preprocessBegin = std::chrono::steady_clock::now();

    // Read depth image
    cv::Mat depth_img = depth_img_ptr->image;
    // int rows = depth_img.rows;
    // int cols = depth_img.cols;

    // Pre-process depth image
    cv::Mat depth_img_preprocessed;
        
    if (hardware_)
    {
        // bilateral filter
        for (int i = 0; i < params.bilat_filter_num_iters; i++)
        {
            cv::Mat temp_img; 
            cv::bilateralFilter(depth_img, 
                                temp_img, 
                                params.bilat_filter_kernel_size, 
                                params.bilat_filter_sigma_color, 
                                params.bilat_filter_sigma_space);
            depth_img = temp_img;
        }


        // shadow infill
        cv::Mat shadow_infill_kernel = cv::Mat::ones(params.infill_filter_kernel_size, params.infill_filter_kernel_size, CV_32F);

        cv::dilate(depth_img, depth_img_preprocessed, shadow_infill_kernel);

    } else
    {
        depth_img_preprocessed = depth_img.clone();
        // cv::bilateralFilter(depth_img, 
        //                     depth_img_preprocessed, 
        //                     params.bilat_filter_kernel_size, 
        //                     params.bilat_filter_sigma_color, 
        //                     params.bilat_filter_sigma_space,
        //                     cv::BORDER_REPLICATE);
    }

    // Publish filtered depth image
    // cv_bridge::CvImagePtr filtered_depth_ptr(new cv_bridge::CvImage);
    filtered_depth_ptr->header = depth_img_ptr->header;
    filtered_depth_ptr->encoding = "32FC1";
    filtered_depth_ptr->image = depth_img_preprocessed;

    filtered_depth_pub.publish(filtered_depth_ptr->toImageMsg());

    // preprocessEnd = std::chrono::steady_clock::now();
    // preprocessTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(preprocessEnd - preprocessBegin).count();
    // numberOfPreprocessCalls++;

    // Normals
    // set size as h x w x 3
    // cv_bridge::CvImagePtr normals_ptr(new cv_bridge::CvImage);
    normals_ptr->header = depth_img_ptr->header;                                
    normals_ptr->encoding = "32FC3";                                            
    normals_ptr->image = cv::Mat(depth_img.rows, depth_img.cols, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

    estimateNormals(depth_img_preprocessed, normals_ptr);

    // checkSparsity(depth_img_preprocessed, normals_ptr);

    // Query middle normal
    // int r = rows / 2;
    // int c = cols / 2;
    // ROS_INFO("      middle normal (%i, %c): %f %f %f", r, c, normals_ptr->image.at<cv::Vec3f>(r, c)[0], 
    //                                                             normals_ptr->image.at<cv::Vec3f>(r, c)[1], 
    //                                                             normals_ptr->image.at<cv::Vec3f>(r, c)[2]);

    // cv_bridge::CvImagePtr normals_bgr_ptr(new cv_bridge::CvImage);
    normals_bgr_ptr->header = depth_img_ptr->header;                                
    normals_bgr_ptr->encoding = "rgb8";                                            
    normals_bgr_ptr->image = normals_ptr->image.clone();

    // Convert from float to 8UC3
    // First, take abs value of normals
    normals_bgr_ptr->image = cv::abs(normals_bgr_ptr->image);

    // Then, convert to 8UC3
    normals_bgr_ptr->image.convertTo(normals_bgr_ptr->image, CV_8UC3, 255.0);

    // ROS_INFO("      middle colored normal: %d %d %d", normals_bgr_ptr->image.at<cv::Vec3b>(r, c)[0], 
    //                                                     normals_bgr_ptr->image.at<cv::Vec3b>(r, c)[1], 
    //                                                     normals_bgr_ptr->image.at<cv::Vec3b>(r, c)[2]);

    // Display normals
    publishNormals(normals_ptr, normals_bgr_ptr);

    // totalEnd = std::chrono::steady_clock::now();
    // totalTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(totalEnd - totalBegin).count();
    // numberOfTotalCalls++;

    // log();

    return;
}

void NormalEstimator::publishNormals(const cv_bridge::CvImagePtr& normals, const cv_bridge::CvImagePtr& normals_bgr)
{
    normals_pub.publish(normals->toImageMsg());
    normals_bgr_img_pub.publish(normals_bgr->toImageMsg());
}

void NormalEstimator::estimateNormals(const cv::Mat& depth_img, cv_bridge::CvImagePtr& normals)
{
    // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), " [NormalEstimator::estimateNormals]");

    // paddingBegin = std::chrono::steady_clock::now();

    // pad depth image by 1 pixel so we can calculate normals for last row/col
    cv::copyMakeBorder(depth_img, depth_img_padded, 0, 1, 0, 1, cv::BORDER_CONSTANT, 0);

    float scale = 0.001; // 1000;
    float inv_scale = 1.0 / scale;

    // Normal estimation code

    int rows = depth_img.rows;
    int cols = depth_img.cols;
    float Z = 0.0, Z_r = 0.0, Z_c = 0.0;
    float dZ_dx = 0.0, dZ_dy = 0.0;

    // int padded_rows = depth_img_padded.rows;
    // int padded_cols = depth_img_padded.cols;

    // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Padding ...");
    // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "  rows: " << rows << ", cols: " << cols);
    // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "  padded_rows: " << padded_rows << ", padded_cols: " << padded_cols);
    for (int c = 0; c < cols; c++)
    {
        // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      col: " << c);
        Z = depth_img.at<float>(rows - 2, c) * scale;
        Z_r = depth_img.at<float>(rows - 1, c) * scale;

        // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      depth at pixel: (" << rows - 2 << ", " << c << ") is: " << Z);
        // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      depth at pixel: (" << rows - 1 << ", " << c << ") is: " << Z_r);

        // Calculate depth gradient
        dZ_dy = (Z_r - Z);

        depth_img_padded.at<float>(rows, c) = (Z_r + dZ_dy) * inv_scale;

        // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      depth at pixel: (" << rows << ", " << c << ") is: " << depth_img_padded.at<float>(rows, c));

    }

    for (int r = 0; r < rows; r++)
    {
        Z = depth_img.at<float>(r, cols - 2) * scale;
        Z_c = depth_img.at<float>(r, cols - 1) * scale;

        // Calculate depth gradient
        dZ_dx = (Z_c - Z);

        depth_img_padded.at<float>(r, cols) = (Z_c + dZ_dx) * inv_scale;
    }

    // paddingEnd = std::chrono::steady_clock::now();
    // paddingTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(paddingEnd - paddingBegin).count();
    // numberOfPaddingCalls++;


    // depthGradientsBegin = std::chrono::steady_clock::now();
    float dX_dx = 0.0, dY_dx = 0.0;
    float dX_dy = 0.0, dY_dy = 0.0;
    Eigen::Vector3f v_x(0.0, 0.0, 0.0), v_y(0.0, 0.0, 0.0), n(0.0, 0.0, 0.0);

    // #pragma omp parallel for collapse(2) // parallelize the loop for better performance
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            // ROS_INFO("      pixel: (%d, %d)", r, c);

            // Do something
            Z = depth_img_padded.at<float>(r, c) * scale;

            if (std::fabs(Z) < DELTA)
            {
                // RCLCPP_WARN_STREAM(nodePtr_->get_logger(), "      zero depth at pixel: (" << r << ", " << c << ")");
                continue;
            }

            Z_r = depth_img_padded.at<float>(r + 1, c) * scale;
            Z_c = depth_img_padded.at<float>(r, c + 1) * scale;

            // Calculate depth gradient
            dZ_dx = (Z_c - Z);
            dZ_dy = (Z_r - Z);

            if (std::abs(dZ_dx) > params.depth_thresh || std::abs(dZ_dy) > params.depth_thresh)
            {
                // RCLCPP_WARN_STREAM(nodePtr_->get_logger(), "      depth gradient too large at pixel: (" << r << ", " << c << "), dZ_dx: " << dZ_dx << ", dZ_dy: " << dZ_dy);
                continue;
            }
            // Calculate X/Y gradients
            dX_dx = (Z * camera.inv_fx) + dZ_dx * (c - camera.u_0) * camera.inv_fx;
            dY_dx = dZ_dx * (r - camera.v_0) * camera.inv_fy;

            dX_dy = dZ_dy * (c - camera.u_0) * camera.inv_fx;
            dY_dy = (Z * camera.inv_fy) + dZ_dy * (r - camera.v_0) * camera.inv_fy;

            // Calculate direcitonal derivatives
            v_x << dX_dx, dY_dx, dZ_dx;
            v_y << dX_dy, dY_dy, dZ_dy;

            // Calculate normal
            n = v_y.cross(v_x); // I think it should be y cross x

            // bool zero_depth = std::fabs(Z) < DELTA;
            // bool zero_normal = n.norm() < DELTA;

            // if (zero_normal && !zero_depth)
            // {
            //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      zero normal at pixel: (" << r << ", " << c << ")");
            //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      Z: " << Z);
            //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      Z_r: " << Z_r);
            //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      Z_c: " << Z_c);
            //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      v_x: " << v_x(0) << " " << v_x(1) << " " << v_x(2));
            //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      v_y: " << v_y(0) << " " << v_y(1) << " " << v_y(2));
            // }

            // Normalize normal
            n.normalize();

            // Set normal
            normals->image.at<cv::Vec3f>(r, c)[0] = n(0);
            normals->image.at<cv::Vec3f>(r, c)[1] = n(1);
            normals->image.at<cv::Vec3f>(r, c)[2] = n(2);
        }
    }

    // depthGradientsEnd = std::chrono::steady_clock::now();
    // depthGradientsTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(depthGradientsEnd - depthGradientsBegin).count();
    // numberOfDepthGradientsCalls++;

    // // Check last row and last col
    // for (int c = 0; c < cols; c++)
    // {
    //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      checking last row, col: " << c);
    //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "        normal at (" << rows - 2 << ", " << c << "): " 
    //                                     << normals->image.at<cv::Vec3f>(rows - 2, c)[0] << " "
    //                                     << normals->image.at<cv::Vec3f>(rows - 2, c)[1] << " "
    //                                     << normals->image.at<cv::Vec3f>(rows - 2, c)[2]);
    //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "        normal at (" << rows - 1 << ", " << c << "): " 
    //                                     << normals->image.at<cv::Vec3f>(rows - 1, c)[0] << " "
    //                                     << normals->image.at<cv::Vec3f>(rows - 1, c)[1] << " "
    //                                     << normals->image.at<cv::Vec3f>(rows - 1, c)[2]);
    // }

    // for (int r = 0; r < rows; r++)
    // {
    //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "      checking last col, row: " << r);
    //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "        normal at (" << r << ", " << cols - 2 << "): " 
    //                                     << normals->image.at<cv::Vec3f>(r, cols - 2)[0] << " "
    //                                     << normals->image.at<cv::Vec3f>(r, cols - 2)[1] << " "
    //                                     << normals->image.at<cv::Vec3f>(r, cols - 2)[2]);
    //     RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "        normal at (" << r << ", " << cols - 1 << "): " 
    //                                     << normals->image.at<cv::Vec3f>(r, cols - 1)[0] << " "
    //                                     << normals->image.at<cv::Vec3f>(r, cols - 1)[1] << " "
    //                                     << normals->image.at<cv::Vec3f>(r, cols - 1)[2]);
    // }

    // take penultimate row/col and copy to last row/col
    // normals->image.row(rows - 1) = normals->image.row(rows - 2).clone();
    // normals->image.col(cols - 1) = normals->image.col(cols - 2).clone();

    // for (int c = 0; c < cols; c++)
    // {
    //     normals->image.at<cv::Vec3b>(rows - 1, c)[0] = normals->image.at<cv::Vec3b>(rows - 2, c)[0];
    //     normals->image.at<cv::Vec3b>(rows - 1, c)[1] = normals->image.at<cv::Vec3b>(rows - 2, c)[1];
    //     normals->image.at<cv::Vec3b>(rows - 1, c)[2] = normals->image.at<cv::Vec3b>(rows - 2, c)[2];
    // }

    // for (int r = 0; r < rows; r++)
    // {
    //     normals->image.at<cv::Vec3b>(r, cols - 1)[0] = normals->image.at<cv::Vec3b>(r, cols - 2)[0];
    //     normals->image.at<cv::Vec3b>(r, cols - 1)[1] = normals->image.at<cv::Vec3b>(r, cols - 2)[1];
    //     normals->image.at<cv::Vec3b>(r, cols - 1)[2] = normals->image.at<cv::Vec3b>(r, cols - 2)[2];
    // }

    return;
}

void NormalEstimator::checkSparsity(const cv::Mat& depth_img, cv_bridge::CvImagePtr& normals_ptr)
{
    int rows = depth_img.rows;
    int cols = depth_img.cols;

    int nan_depth_count = 0;
    int nan_normal_count = 0;

    int finite_depth_count = 0;
    int finite_normal_count = 0;

    int zero_depth_count = 0;
    int zero_normal_count = 0;

    int depth_flag = -1;
    int normal_flag = -1;

    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            depth_flag = -1;
            normal_flag = -1; 

            if (depth_img.at<float>(r, c) != depth_img.at<float>(r, c))
            {
                // RCLCPP_ERROR_STREAM(nodePtr_->get_logger(), "Depth image has NaN value at row: " << r << ", col: " << c);
                nan_depth_count++;
                depth_flag = 0;
            } else if (std::fabs(depth_img.at<float>(r, c)) < DELTA)
            {
                zero_depth_count++;
                depth_flag = 1;
            } else
            {
                // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Depth image value at row: " << r << ", col: " << c << " is: " << depth_img.at<float>(r, c));
                finite_depth_count++;
                depth_flag = 2;
            }

            if (normals_ptr->image.at<cv::Vec3f>(r, c)[0] != normals_ptr->image.at<cv::Vec3f>(r, c)[0] ||
                normals_ptr->image.at<cv::Vec3f>(r, c)[1] != normals_ptr->image.at<cv::Vec3f>(r, c)[1] ||
                normals_ptr->image.at<cv::Vec3f>(r, c)[2] != normals_ptr->image.at<cv::Vec3f>(r, c)[2])
            {
                // RCLCPP_ERROR_STREAM(nodePtr_->get_logger(), "Normal image has NaN value at row: " << r << ", col: " << c);
                nan_normal_count++;
                normal_flag = 0;
            } else if (  cv::norm(normals_ptr->image.at<cv::Vec3f>(r, c)) < DELTA)
            {
                zero_normal_count++;
                normal_flag = 1;
            } else
            {
                // RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Normal image value at row: " << r << ", col: " << c << " is: " << normals_ptr->image.at<cv::Vec3f>(r, c));
                finite_normal_count++;
                normal_flag = 2;
            }

            if (depth_flag != normal_flag)
            {
                RCLCPP_ERROR_STREAM(nodePtr_->get_logger(), "Depth and normal image values do not match at row: " << r << ", col: " << c);
                RCLCPP_ERROR_STREAM(nodePtr_->get_logger(), "Depth flag: " << depth_flag << ", Normal flag: " << normal_flag);
            }
        }
    }


    // int total_pixels = rows * cols;

    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Depth image NaN count: " << nan_depth_count);
    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Depth image zero count: " << zero_depth_count);
    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Depth image finite count: " << finite_depth_count);

    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Normal image NaN count: " << nan_normal_count);
    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Normal image zero count: " << zero_normal_count);
    RCLCPP_INFO_STREAM(nodePtr_->get_logger(), "Normal image finite count: " << finite_normal_count);
}
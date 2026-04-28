#include <superpixels/ImagePreprocessor.h>

ImagePreprocessor::ImagePreprocessor(const SuperpixelParams & params, const rclcpp::Node::SharedPtr & node)
{
    params_ = params;
    node_ = node;

    // tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void ImagePreprocessor::setParams(const SuperpixelParams & params)
{
    params_ = params;
}

void ImagePreprocessor::cleanImages(const cv::Mat & raw_depth_img,
                                    const cv::Mat & raw_normal_img,
                                    cv::Mat & cleaned_depth_img,
                                    cv::Mat & cleaned_normal_img,
                                    cv::Mat & visited)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::cleanImages]");

    // Depth
    cleaned_depth_img = cv::Mat(raw_depth_img.size(), CV_32F, cv::Scalar(0));

    // Normals
    cleaned_normal_img = cv::Mat(raw_normal_img.size(), CV_32FC3, cv::Scalar(0));

    // zero invalid depth pixels
    for (int r = 0; r < cleaned_depth_img.rows; r++)
    {
        for (int c = 0; c < cleaned_depth_img.cols; c++)
        {
            if (isPixelValid(raw_depth_img, raw_normal_img, cv::Point(c, r), params_.k_c_)) // raw_label_img, 
            {

                float depth = raw_depth_img.at<float>(r, c);

                // bool valid_depth = (!std::isnan(depth) && std::abs(depth) > 1e-6 && depth > 0);

                cv::Vec3f normal = raw_normal_img.at<cv::Vec3f>(r, c);

                // bool valid_normal = (!std::isnan(normal.val[0]) && !std::isnan(normal.val[1]) && !std::isnan(normal.val[2]) && 
                                        // cv::norm(normal) > DELTA);

                // if (valid_depth && valid_label && valid_normal)
                // {
                cleaned_depth_img.at<float>(r, c) = depth;
                // cleaned_label_img.at<uint8_t>(r, c) = label;
                cleaned_normal_img.at<cv::Vec3f>(r, c) = normal;
                visited.at<uint8_t>(r, c) = 1;
                // } 
            } else
            {
                cleaned_depth_img.at<float>(r, c) = 0;
                // cleaned_label_img.at<uint8_t>(r, c) = 0;
                cleaned_normal_img.at<cv::Vec3f>(r, c) = cv::Vec3f(0, 0, 0);
            }

            // if (std::isnan(depth))
            // {
            //     cleaned_depth_img.at<float>(r, c) = 0;
            // } else if (std::fabs(depth) < 1e-6)
            // {
            //     cleaned_depth_img.at<float>(r, c) = 0;
            // } else if (depth < 0)
            // {
            //     cleaned_depth_img.at<float>(r, c) = 0;
            // } else
            // {
            //     cleaned_depth_img.at<float>(r, c) = depth;
            //     visited_.at<uint8_t>(r, c) = 1;
            // }
    //     }
    // }    

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Checking label image ...");

    // for (int r = 0; r < cleaned_label_img.rows; r++)
    // {
    //     for (int c = 0; c < cleaned_label_img.cols; c++)
    //     {

            // if (std::isnan(label))
            //     cleaned_label_img.at<uint8_t>(r, c) = 0;
            // else if (label < 0)
            //     cleaned_label_img.at<uint8_t>(r, c) = 0;
            // else
            //     cleaned_label_img.at<uint8_t>(r, c) = label;
    //     }
    // }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Checking normal image ...");

    // for (int r = 0; r < cleaned_normal_img.rows; r++)
    // {
    //     for (int c = 0; c < cleaned_normal_img.cols; c++)
    //     {

            // if (std::isnan(normal.val[0]) || std::isnan(normal.val[1]) || std::isnan(normal.val[2]))
            //     cleaned_normal_img.at<cv::Vec3f>(r, c) = cv::Vec3f(0, 0, 0);
            // else if (cv::norm(normal) < DELTA)
            //     cleaned_normal_img.at<cv::Vec3f>(r, c) = cv::Vec3f(0, 0, 0);
            // else
            //     cleaned_normal_img.at<cv::Vec3f>(r, c) = normal;
        }
    }

    return;
}


void ImagePreprocessor::healthCheck(const cv::Mat & depth_img,
                                    // const cv::Mat & label_img,
                                    const cv::Mat & normal_img)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::healthCheck]");
    // Detecting invalid pixels

    int nan_depth_count = 0;
    int negative_depth_count = 0;
    int finite_depth_count = 0;
    int zero_depth_count = 0;

    // int nan_label_count = 0;
    // int negative_label_count = 0;
    // int finite_label_count = 0;

    int nan_normal_count = 0;
    int finite_normal_count = 0;
    int zero_normal_count = 0;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "        Checking images ...");

    // Depth
    for (int r = 0; r < depth_img.rows; r++)
    {
        for (int c = 0; c < depth_img.cols; c++)
        {
            float depth = depth_img.at<float>(r, c);

            if (std::isnan(depth))
            {
                RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), nan_depth_count == 0, "            Detected NaN depth pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
                nan_depth_count++;
            } else if (std::abs(depth) < 1e-6)
            {
                RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), zero_depth_count == 0, "            Detected zero depth pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
                zero_depth_count++;
            } else if (depth < 0)
            {
                RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), negative_depth_count == 0, "            Detected negative depth pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
                negative_depth_count++;
            } else
            {
                finite_depth_count++;
            }

            // uint8_t label = label_img.at<uint8_t>(r, c);

            // if (std::isnan(label))
            // {
            //     RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), nan_label_count == 0, "            Detected NaN label pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
            //     nan_label_count++;
            // } else if (label < 0)
            // {
            //     RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), negative_label_count == 0, "            Detected negative label pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
            //     negative_label_count++;
            // } else
            // {
            //     finite_label_count++;
            // }

            cv::Vec3f normal = normal_img.at<cv::Vec3f>(r, c);

            if (std::isnan(normal.val[0]) || std::isnan(normal.val[1]) || std::isnan(normal.val[2]))
            {
                RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), nan_normal_count == 0, "            Detected NaN normal pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
                nan_normal_count++;
            } else if (cv::norm(normal) < DELTA)
            {
                RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), zero_normal_count == 0, "            Detected zero normal pixel"); //  at: (" << r << ", " << c << "), zeroing ...");
                zero_normal_count++;
            } else
            {
                finite_normal_count++;
            }
        }
    }

    int total_pixels = depth_img.rows * depth_img.cols;

    RCLCPP_INFO_STREAM(node_->get_logger(), "        Health check:");
    RCLCPP_INFO_STREAM(node_->get_logger(), "           Depth:");
    RCLCPP_INFO_STREAM(node_->get_logger(), "               NaN count: " << nan_depth_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Negative count: " << negative_depth_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Zero count: " << zero_depth_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Finite count: " << finite_depth_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Unaccounted for: " << total_pixels - (nan_depth_count + negative_depth_count + zero_depth_count + finite_depth_count));
    // RCLCPP_INFO_STREAM(node_->get_logger(), "        Label:");
    // RCLCPP_INFO_STREAM(node_->get_logger(), "               NaN count: " << nan_label_count);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "               Negative count: " << negative_label_count);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "               Finite count: " << finite_label_count);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "               Unaccounted for: " << total_pixels - (nan_label_count + negative_label_count + finite_label_count));
    RCLCPP_INFO_STREAM(node_->get_logger(), "        Normal:");
    RCLCPP_INFO_STREAM(node_->get_logger(), "               NaN count: " << nan_normal_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Zero count: " << zero_normal_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Finite count: " << finite_normal_count);
    RCLCPP_INFO_STREAM(node_->get_logger(), "               Unaccounted for: " << total_pixels - (nan_normal_count + zero_normal_count + finite_normal_count));

    if (finite_depth_count != finite_normal_count)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "        Finite counts do not match between depth and normal images.");
    }

}

void ImagePreprocessor::preprocessImages(const cv::Mat & cleaned_depth_img,
                                            // const cv::Mat & cleaned_label_img,
                                            const cv::Mat & cleaned_normal_img,
                                            cv::Mat & preprocessed_depth_img,
                                            // cv::Mat & preprocessed_label_img,
                                            cv::Mat & preprocessed_normal_img)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::preprocessImages]");

    if (params_.kernel_radius_ < 1)
    {
        preprocessed_depth_img = cleaned_depth_img.clone();
        // preprocessed_label_img = cleaned_label_img.clone();
        preprocessed_normal_img = cleaned_normal_img.clone();
        return;
    }

    // Custom dilation implementation
    // float max_depth = -std::numeric_limits<float>::max();
    // int max_label = 0;
    // cv::Vec3f max_normal(0, 0, 0);
    // cv::Point max_depth_pixel(-1, -1);

    float min_depth = std::numeric_limits<float>::max();
    // int min_label = 0;
    cv::Vec3f min_normal(0, 0, 0);
    cv::Point min_depth_pixel(-1, -1);

    cv::Mat dilated_depth_img = cleaned_depth_img.clone(); // cv::Mat(cleaned_depth_img.size(), CV_32F, cv::Scalar(0));
    // cv::Mat dilated_label_img = cleaned_label_img.clone(); // cv::Mat(cleaned_label_img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat dilated_normal_img = cleaned_normal_img.clone(); // cv::Mat(cleaned_normal_img.size(), CV_32FC3, cv::Scalar(0));

    cv::Mat temp_dilated_depth_img = cv::Mat(cleaned_depth_img.size(), CV_32F, cv::Scalar(0));
    // cv::Mat temp_dilated_label_img = cv::Mat(cleaned_label_img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp_dilated_normal_img = cv::Mat(cleaned_normal_img.size(), CV_32FC3, cv::Scalar(0));

    int new_r = 0;
    int new_c = 0;

    float depth = 0;

    for (int iter = 0; iter < params_.num_dilation_iterations_; iter++)
    {
        for (int r = 0; r < cleaned_depth_img.rows; r++)
        {
            for (int c = 0; c < cleaned_depth_img.cols; c++)
            {
                // max_depth = 0;
                // max_label = 0;
                // max_normal = cv::Vec3f(0, 0, 0);
                // max_depth_pixel = cv::Point(-1, -1);

                min_depth = std::numeric_limits<float>::max();
                // min_label = 0;
                min_normal = cv::Vec3f(0, 0, 0);
                min_depth_pixel = cv::Point(-1, -1);

                for (int i = -params_.kernel_radius_; i <= params_.kernel_radius_; i++)
                {
                    for (int j = -params_.kernel_radius_; j <= params_.kernel_radius_; j++)
                    {
                        new_r = r + i;
                        new_c = c + j;

                        if (!isPixelInBounds(params_.k_c_, cv::Point(new_c, new_r)))
                        {
                            continue;
                        }

                        depth = dilated_depth_img.at<float>(new_r, new_c);

                        if (depth > 0.0 && depth < min_depth)
                        {
                            min_depth = depth;
                            // min_label = dilated_label_img.at<uint8_t>(new_r, new_c);
                            min_normal = dilated_normal_img.at<cv::Vec3f>(new_r, new_c);
                            min_depth_pixel = cv::Point(new_c, new_r);
                            temp_dilated_depth_img.at<float>(r, c) = min_depth;
                            // temp_dilated_label_img.at<uint8_t>(r, c) = min_label;
                            temp_dilated_normal_img.at<cv::Vec3f>(r, c) = min_normal;                            
                        }

                        // if (depth > max_depth)
                        // {
                        //     max_depth = depth;
                        //     max_label = dilated_label_img.at<uint8_t>(new_r, new_c);
                        //     max_normal = dilated_normal_img.at<cv::Vec3f>(new_r, new_c);
                        //     max_depth_pixel = cv::Point(new_c, new_r);
                        // }
                    }
                }

                // temp_dilated_depth_img.at<float>(r, c) = max_depth;
                // temp_dilated_label_img.at<uint8_t>(r, c) = max_label;
                // temp_dilated_normal_img.at<cv::Vec3f>(r, c) = max_normal;

                // temp_dilated_depth_img.at<float>(r, c) = min_depth;
                // temp_dilated_label_img.at<uint8_t>(r, c) = min_label;
                // temp_dilated_normal_img.at<cv::Vec3f>(r, c) = min_normal;
            }
        }

        dilated_depth_img = temp_dilated_depth_img.clone();
        // dilated_label_img = temp_dilated_label_img.clone();
        dilated_normal_img = temp_dilated_normal_img.clone();
    }

    preprocessed_depth_img = dilated_depth_img.clone();
    // preprocessed_label_img = dilated_label_img.clone();
    preprocessed_normal_img = dilated_normal_img.clone();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Comparing normal image to dilated normal image ...");
    // Compare normal image with dilated normal image
    // for (int r = 0; r < cleaned_normal_img.rows; r++)
    // {
    //     for (int c = 0; c < cleaned_normal_img.cols; c++)
    //     {
    //         cv::Vec3f normal = cleaned_normal_img.at<cv::Vec3f>(r, c);
    //         // cv::Vec3f added_normal = added_normal_img.at<cv::Vec3f>(r, c);

    //         if (cv::norm(normal) < DELTA)
    //         {
    //             continue;
    //         }


    //         cv::Vec3f dilated_normal = preprocessed_normal_img.at<cv::Vec3f>(r, c);

    //         RCLCPP_INFO_STREAM(node_->get_logger(), "       Normal: (" << normal.val[0] << ", " << normal.val[1] << ", " << normal.val[2] << ")");
    //         // RCLCPP_INFO_STREAM(node_->get_logger(), "       Added normal: (" << added_normal.val[0] << ", " << added_normal.val[1] << ", " << added_normal.val[2] << ")");
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "       Dilated Normal: (" << dilated_normal.val[0] << ", " << dilated_normal.val[1] << ", " << dilated_normal.val[2] << ")");
            
    //     }
    // }    
}

//                                     const cv_bridge::CvImagePtr & raw_depth_img_ptr
void ImagePreprocessor::fillInImage(const cv::Mat & cleaned_depth_img,
                                    // const cv::Mat & cleaned_label_img,
                                    const cv::Mat & cleaned_normal_img,
                                    const cv::Mat & visited,
                                    cv::Mat & filled_depth_img,
                                    // cv::Mat & filled_label_img,
                                    cv::Mat & filled_normal_img,
                                    const double & default_height)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelDepthSegmenter::fillInImage]");

    // double default_height = 0.40;

    generator.seed(123456789);
    std::normal_distribution<double> distribution(0.0, 0.00001);

  // generator.seed(std::hash<std::string>{}(regionID));

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       egocanFrameToWorldFrame: " << egocanFrameToWorldFrame);

    // Depth
    filled_depth_img = cleaned_depth_img.clone();

    // Labels
    // filled_label_img = cleaned_label_img.clone();

    // Normals
    filled_normal_img = cleaned_normal_img.clone();

    int delta = params_.step_ / 8; // bit heuristic, close to actual image sparsity

    for (int r = delta; r < (cleaned_depth_img.rows - delta); r += delta)
    {
        for (int c = delta; c < (cleaned_depth_img.cols - delta); c += delta)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "    Checking pixel: (" << r << ", " << c << ")");

            cv::Point curr_pixel(c, r);
            if (isPixelInBounds(params_.k_c_, curr_pixel) )
            {
                bool found_visited_pixel = false;

                for (int i = -delta; i <= delta; i++)
                {
                    for (int j = -delta; j <= delta; j++)
                    {
                        cv::Point pixel(c + j, r + i);

                        if (isPixelInBounds(params_.k_c_, pixel) && visited.at<uint8_t>(pixel.y, pixel.x) == 1)
                        {
                            found_visited_pixel = true;
                            break;
                        }
                    }

                    if (found_visited_pixel)
                    {
                        break;
                    }
                }

                if (!found_visited_pixel)
                {
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "       did not find a pixel, filling in at (" << r << ", " << c << ") ...");

                    filled_depth_img.at<float>(r, c) = default_height + distribution(generator);
                    // filled_label_img.at<uint8_t>(r, c) = 3;
                    filled_normal_img.at<cv::Vec3f>(r, c) = cv::Vec3f(0, -1.0, 0);
                } else
                {
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "       invalid");
                }
            }
        }
    }
}
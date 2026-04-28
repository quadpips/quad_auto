#include <superpixels/Visualizer.h>

Visualizer::Visualizer(const SuperpixelParams & params, const rclcpp::Node::SharedPtr & node)
{
    params_ = params;
    node_ = node;

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // Set up subscribers and publishers
    image_transport::ImageTransport it(node_);

    fin_depth_img_pub_ = it.advertise("/superpixels/process_depth", 1);
    fin_depth_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

    // fin_label_img_pub_ = it.advertise("/superpixels/process_labels", 1);
    // fin_label_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

    fin_normal_img_pub_ = it.advertise("/superpixels/process_normals", 1);
    fin_normal_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
    fin_normal_img_colored_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

    center_grid_img_pub_ = it.advertise("/superpixels/center_grid", 1);
    center_grid_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

    colored_cluster_img_pub_ = it.advertise("/superpixels/colored_clusters", 1);
    colored_cluster_img_ptr_ = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

    // colored_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/superpixels/colored_point_cloud", 1);
    // colored_centroids_pub_ = nh.advertise<visualization_msgs::msg::MarkerArray>("/superpixels/colored_centroids", 1);
    colored_point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/superpixels/colored_point_cloud", 1);
    colored_centroids_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/superpixels/colored_centroids", 1);
    terrainPub_ = node_->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>("/convex_plane_decomposition_ros/planar_terrain", 1);

    localRegionPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/superpixels/planar_regions", 1);
    localRegionNormalPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/superpixels/planar_region_normals", 1);
    localRegionIDPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/superpixels/ids", 1);

    elevationMapPublisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("/superpixels/elevation_map", 1);

    setColors();    

    // set up terrain publisher
    // terrainPub_ = nh.advertise<convex_plane_decomposition_msgs::PlanarTerrain>
                                    // ("/convex_plane_decomposition_ros/planar_terrain", 1);
}

void Visualizer::setParams(const SuperpixelParams & params)
{
    params_ = params;
    setColors();
}

void Visualizer::setColors()
{
    colors_.clear();
    colors_.resize(params_.num_superpixels_);
    for (int i = 0; i < (int) colors_.size(); i++)
    {
        cv::Scalar color(rand() % 255, rand() % 255, rand() % 255);

        // for (int j = 0; j < i; j++)
        // {
        //     if (cv::norm(color - colors_[j]) < 50)
        //     {
        //         color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        //         j = -1;
        //     }
        // }

        colors_[i] = color;
    }
}

void Visualizer::visualize(const cv::Mat & depth_image,
                            const cv::Mat & normal_image,
                            const cv_bridge::CvImagePtr & raw_depth_img_ptr,
                            const cv_bridge::CvImagePtr & raw_normal_img_ptr,
                            const std::vector<std::vector<double>> & centers,
                            const cv::Mat & clusters,
                            const std::vector<int> & center_counts,
                            const std::vector<std::vector<Eigen::Vector2d>> & superpixel_projections,
                            const std::vector<std::vector<Eigen::Vector2d>> & superpixel_convex_hulls,
                            const std::vector<Eigen::Matrix3d> & egocan_to_region_rotations)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [Visualizer::visualize]");

    // WARNING: clusters IDs are wrong after removing superpixels, ID itself is invalid

    if (centers.size() != center_counts.size())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "   [Visualizer::visualize] centers size (" << centers.size() << ") and center_counts size (" << center_counts.size() << ") do not match!");
        return;
    }

    if (centers.size() != superpixel_projections.size())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "   [Visualizer::visualize] centers size (" << centers.size() << ") and superpixel_projections size (" << superpixel_projections.size() << ") do not match!");
        return;
    }

    if (centers.size() != superpixel_convex_hulls.size())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "   [Visualizer::visualize] centers size (" << centers.size() << ") and superpixel_convex_hulls size (" << superpixel_convex_hulls.size() << ") do not match!");
        return;
    }

    if (centers.size() != egocan_to_region_rotations.size())
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "   [Visualizer::visualize] centers size (" << centers.size() << ") and egocan_to_region_rotations size (" << egocan_to_region_rotations.size() << ") do not match!");
        return;
    }

    // std::lock_guard<std::mutex> lock(img_mutex_);

    // if (notReceivedImage())
    // {
    //     ROS_WARN("Not ready to visualize, no images received yet.");
    //     return;
    // }

    cv::Mat raw_depth_image = raw_depth_img_ptr->image;
    cv::Mat raw_normal_image = raw_normal_img_ptr->image;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Publishing final depth image");
    fin_depth_img_ptr_->header = raw_depth_img_ptr->header;
    fin_depth_img_ptr_->encoding = raw_depth_img_ptr->encoding;
    cv::Mat flipped_depth_image;
    cv::flip(depth_image, flipped_depth_image, 1); // flip horizontally
    fin_depth_img_ptr_->image = flipped_depth_image;
    fin_depth_img_pub_.publish(fin_depth_img_ptr_->toImageMsg());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Preparing final label image");
    // fin_label_img_ptr_->header = raw_label_img_ptr->header;
    // fin_label_img_ptr_->encoding = raw_label_img_ptr->encoding;
    // fin_label_img_ptr_->image = label_image;
    // fin_label_img_pub_->publish(fin_label_img_ptr_->toImageMsg());

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Preparing final normal image");
    // fin_normal_img_ptr_->header = raw_normal_img_ptr->header;
    // fin_normal_img_ptr_->encoding = raw_normal_img_ptr->encoding;
    // fin_normal_img_ptr_->image = normal_image;
    // No publishing normal image

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Publishing final colored normal image");
    fin_normal_img_colored_ptr_->header = raw_normal_img_ptr->header;
    fin_normal_img_colored_ptr_->encoding = "rgb8";
    cv::Mat colored_normal_image = normal_image;
    // fin_normal_img_colored_ptr_->image = fin_normal_img_ptr_->image;
    cv::Mat abs_colored_normal_image = cv::abs(colored_normal_image);
    cv::Mat scaled_colored_normal_image = abs_colored_normal_image;
    scaled_colored_normal_image.convertTo(scaled_colored_normal_image, CV_8UC3, 255.0);
    cv::Mat flipped_scaled_colored_normal_image;
    cv::flip(scaled_colored_normal_image, flipped_scaled_colored_normal_image, 1); // flip horizontally
    fin_normal_img_colored_ptr_->image = flipped_scaled_colored_normal_image;
    fin_normal_img_pub_.publish(fin_normal_img_colored_ptr_->toImageMsg());

    // cv::Mat color_depth_image = cv::Mat(depth_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    // convertDepthImageToColor(color_depth_image, depth_image);

    // overlayCenters(color_depth_image, centers);

    // colorClusters(color_depth_image, clusters);

    colorClusterPointCloud(depth_image, clusters);

    // colorCentroids(centers, center_counts);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Publishing planar regions");

    // depth_image, 
    publishPlanarRegions(raw_depth_image, centers, center_counts, superpixel_convex_hulls, egocan_to_region_rotations);

    // outputToDatFile(raw_depth_img_ptr, superpixel_projections);

    return;
}


// const cv::Mat & depth_img,
void Visualizer::publishPlanarRegions(const cv::Mat & raw_depth_image,
                                        const std::vector<std::vector<double>> & centers,
                                        const std::vector<int> & center_counts,
                                        const std::vector<std::vector<Eigen::Vector2d>> & superpixel_convex_hulls,
                                        const std::vector<Eigen::Matrix3d> & egocan_to_region_rotations)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [Visualizer::publishPlanarRegions]");

    convex_plane_decomposition_msgs::msg::PlanarTerrain terrain_msg;

    rclcpp::Time lookupTime = fin_depth_img_ptr_->header.stamp;
    std::string egocan_frame = fin_depth_img_ptr_->header.frame_id;

    // rclcpp::Duration timeout(3, 0); // 3 seconds
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

    // rclcpp::Duration timeout(3, 0); // 3 seconds
    // geometry_msgs::msg::TransformStamped egocanFrameToEgocanStabilizedFrame;
    // try
    // {
    //     egocanFrameToEgocanStabilizedFrame = tfBuffer_->lookupTransform("egocan_stabilized", egocan_frame, lookupTime); // , timeout
    // }
    // catch (tf2::TransformException & ex)
    // {
    //     RCLCPP_WARN_STREAM(node_->get_logger(), "   [Visualizer::publishPlanarRegions] TF lookup failed: " << ex.what());
    //     return;
    // }


    double foot_radius = 0.02;

    convex_plane_decomposition::PlanarRegion region;
    convex_plane_decomposition::BoundaryWithInset boundaryWithInset;
    convex_plane_decomposition::CgalPolygonWithHoles2d polygonWithHoles;
    convex_plane_decomposition::CgalPolygon2d polygon;
    convex_plane_decomposition::CgalPolygon2d inflated_polygon;
    convex_plane_decomposition::CgalPolygonWithHoles2d inflated_polygon_with_holes;
    convex_plane_decomposition_msgs::msg::PlanarRegion region_msg;

    std::vector<convex_plane_decomposition::CgalPolygonWithHoles2d> insets(1);

    cv::Point center_pixel;
    float center_depth;
    cv::Vec3f centerEgocanCvPt;
    Eigen::Matrix3d regionToEgocanRotMat;
    Eigen::Quaterniond regionToEgocanQuat;
    Eigen::Matrix3d egocanToWorldRotMat;

    Eigen::VectorXd centerWorldPose;

    Eigen::Vector2d convexHullPt, convexHullDir, inflatedConvexHullPt;

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       planar regions:");
    for (size_t i = 0; i < centers.size(); i++)
    {        
        // RCLCPP_INFO_STREAM(node_->get_logger(), "           i: " << i);

        if (center_counts[i] == 0)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "           Region " << i << " has no points.");
            continue;
        }

        center_pixel = cv::Point(centers[i][0], centers[i][1]);
        center_depth = centers[i][2];

        pixelToEgocanFrame(centerEgocanCvPt, center_pixel, center_depth, params_.k_c_, params_.h_);

        Eigen::Vector3d centerEgocanPt(centerEgocanCvPt.val[0], centerEgocanCvPt.val[1], centerEgocanCvPt.val[2]);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "               center (pixel): " << centers[i][0] << ", " << centers[i][1]);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "               center (depth): " << centers[i][2]);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "               center (egocan frame): " << centerEgocanPt.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "               normal (egocan frame): " << centers[i][3] << ", " << centers[i][4] << ", " << centers[i][5]);

        regionToEgocanRotMat = egocan_to_region_rotations[i].transpose();
        regionToEgocanQuat = Eigen::Quaterniond(regionToEgocanRotMat);

        centerWorldPose = transformHelperPoseStamped(centerEgocanPt, regionToEgocanQuat, egocanFrameToOdomFrame);

        // get rotation matrix
        egocanToWorldRotMat = calculateRotationMatrix(centerWorldPose[3], centerWorldPose[4], centerWorldPose[5]);

        region.transformPlaneToWorld.translation() = centerWorldPose.head(3);
        region.transformPlaneToWorld.linear() = egocanToWorldRotMat;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "               translation: " << region.transformPlaneToWorld.translation().transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "               rotation: " << region.transformPlaneToWorld.linear().row(0));
        // RCLCPP_INFO_STREAM(node_->get_logger(), "                         " << region.transformPlaneToWorld.linear().row(1));
        // RCLCPP_INFO_STREAM(node_->get_logger(), "                         " << region.transformPlaneToWorld.linear().row(2));


        // RCLCPP_INFO_STREAM(node_->get_logger(), "               convex hull:");
        polygon.container().clear();
        inflated_polygon.container().clear();
        for (size_t j = 0; j < superpixel_convex_hulls[i].size(); j++)
        {
            convexHullPt = superpixel_convex_hulls[i][j];

            // normal polygon
            // polygon.container().emplace_back(convexHullPt[0], convexHullPt[1]);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "           point " << j << ": " << polygon.container()[j].x() << ", " << polygon.container()[j].y());

            // inflated polygon
            double norm = convexHullPt.norm();
            convexHullDir = convexHullPt / norm;

            if (norm > (2 * foot_radius) )
            {
                convexHullPt = convexHullPt - foot_radius * convexHullDir;
                inflatedConvexHullPt = convexHullPt - 2 * foot_radius * convexHullDir;
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           inflated point " << j << ": " << foot[0] << ", " << foot[1]);
            } else
            {
                convexHullPt = 0.75 * convexHullPt;
                inflatedConvexHullPt = 0.5 * convexHullPt;
                // RCLCPP_INFO_STREAM(node_->get_logger(), "           inflated point " << j << ": " << foot[0] << ", " << foot[1]);
            }
            
            polygon.container().emplace_back(convexHullPt[0], convexHullPt[1]);
            inflated_polygon.container().emplace_back(inflatedConvexHullPt[0], inflatedConvexHullPt[1]);

            // RCLCPP_INFO_STREAM(node_->get_logger(), "           inflated point " << j << ": " << inflated_polygon.container()[j].x() << ", " << inflated_polygon.container()[j].y());
        }

        polygonWithHoles.outer_boundary() = polygon; // inflated_polygon; // 
        boundaryWithInset.boundary = polygonWithHoles;

        inflated_polygon_with_holes.outer_boundary() = inflated_polygon;

        insets[0] = inflated_polygon_with_holes;

        boundaryWithInset.insets = insets;

        region.boundaryWithInset = boundaryWithInset;
        region.bbox2d = boundaryWithInset.boundary.outer_boundary().bbox();

        region_msg = convex_plane_decomposition::toMessage(region);
        // cv::Scalar color = colors_[i];
        std_msgs::msg::ColorRGBA region_color;
        // region_color.r = color[2] / 255.0;
        // region_color.g = color[1] / 255.0;
        // region_color.b = color[0] / 255.0;
        // region_color.r = 0.0;
        // region_color.g = 0.0;
        // region_color.b = 0.0;
        // region_color.a = 1.0;
        // region_msg.color = region_color;

        terrain_msg.planar_regions.push_back(region_msg);

        // RCLCPP_INFO_STREAM(node_->get_logger(), "   e0: " << e0.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "   e1: " << e1.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "   normal: " << normal.transpose());
    }    

    /* Only compare to pixels in a 2 x step by 2 x step region. */
    // grid_map::GridMap map({"elevation"});
    // map.setFrameId("odom");
    // map.setGeometry(grid_map::Length(1.2, 2.0), 0.03);

    // rclcpp::Time time = node_->now();
    // for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    // {
    //   grid_map::Position position;
    //   map.getPosition(*it, position);
    //   map.at(
    //     "elevation",
    //     *it) = -0.04 + 0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()) * position.x();
    // }

    // // Publish grid map.
    // map.setTimestamp(time.nanoseconds());
    // std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    // message = grid_map::GridMapRosConverter::toMessage(map);
    // elevationMapPublisher_->publish(std::move(message));


    grid_map::GridMap map({"elevation"});
    map.setFrameId({"odom"}); // needs to be odom
    // 1 x 1 too small
    float map_width = 3.0;
    float map_length = 3.0;
    float map_resolution = 0.03;

    map.setGeometry(grid_map::Length(map_width, map_length), 
                    map_resolution, 
                    grid_map::Position(egocanFrameToOdomFrame.transform.translation.x, egocanFrameToOdomFrame.transform.translation.y));    
    
    cv::Vec3f egocanPt;
    Eigen::Vector3d egocanEigenPt;
    Eigen::Vector3d worldPt;
    // Eigen::Vector3d egocanStabilizedPt;
    cv::Point current;
    float depth;

    rclcpp::Time time = node_->now();
    for (int r = 0; r < raw_depth_image.rows; r++) 
    {
        // #pragma omp parallel for
        for (int c = 0; c < raw_depth_image.cols; c++) 
        {                
            grid_map::Position map_position;

            // RCLCPP_INFO_STREAM(node_->get_logger(), "       r: " << r << ", c: " << c);

            current = cv::Point(c, r);
            if (isDepthValid(raw_depth_image, current, params_.k_c_)) 
            {
                depth = raw_depth_image.at<float>(r, c);
                pixelToEgocanFrame(egocanPt, current, depth, params_.k_c_, params_.h_);
                egocanEigenPt = Eigen::Vector3d(egocanPt[0], egocanPt[1], egocanPt[2]);

                worldPt = transformHelperPointStamped(egocanEigenPt, egocanFrameToOdomFrame);
                // egocanStabilizedPt = transformHelperPointStamped(egocanEigenPt, egocanFrameToEgocanStabilizedFrame);

                // RCLCPP_INFO_STREAM(node_->get_logger(), "   trying to add worldPt: " << worldPt.transpose());

                map_position.x() = worldPt[0];
                map_position.y() = worldPt[1];

                if (!map.isInside(map_position))
                {
                    // RCLCPP_INFO_STREAM(node_->get_logger(), "   position not inside map at r: " << r << ", c: " << c);
                    continue;
                }                

                map.atPosition("elevation", map_position) = worldPt[2];
            } 
            // else
            // {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "   invalid pixel at r: " << r << ", c: " << c);
            // }
        }
    }

    convex_plane_decomposition::PlaneDecompositionPipeline::Config perceptionConfig;

    convex_plane_decomposition::GridMapPreprocessing preprocessing_(perceptionConfig.preprocessingParameters);

    // preprocess layer
    preprocessing_.preprocess(map, "elevation");

    // add inpaint layer
    // map.add("inpaint", 0.0);

    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    elevationMapPublisher_->publish(std::move(message));

    // const std::string elevationLayer{"elevation"};
    // const std::string frameId = "egocan_stabilized"; // need to stabilized?
    // const float resolution = 0.01; // what is this?
    // const float heightScale = 0.01; // what is this?
    // auto imageGridMap = convex_plane_decomposition::loadGridmapFromImage(depth_image, 
    //                                                                         elevationLayer, 
    //                                                                         frameId,
    //                                                                         resolution, 
    //                                                                         heightScale);
    // grid_map_msgs::msg::GridMap image_grid_map_msg = *grid_map::GridMapRosConverter::toMessage(imageGridMap);
    // elevationMapPublisher_->publish(image_grid_map_msg);

    // const std::string ocs2_anymal = "/home/masselmeier3/ros2_ws/src/ocs2/ocs2_robotic_examples/ocs2_perceptive_quadruped/anymal/";
    // const std::string terrainFolder = ocs2_anymal + "ocs2_anymal_loopshaping_mpc/data/";
    // std::string terrainFile = "step.png";

    // const std::string elevationLayer{"elevation"};
    // const std::string frameId{"odom"};
    // const float resolution = 0.04; // what is this?
    // const float heightScale = 0.35; // what is this?
    // auto gridMap = convex_plane_decomposition::loadGridmapFromImage(
    //     terrainFolder + "/" + terrainFile, elevationLayer, frameId,
    //     resolution, heightScale);
    // gridMap.get(elevationLayer).array() -=
    //     gridMap.atPosition(elevationLayer, {0., 0.});    

    // grid_map_msgs::msg::GridMap elevationMapMessage =
    //     *(grid_map::GridMapRosConverter::toMessage(gridMap));
    // elevationMapPublisher_->publish(elevationMapMessage);

    // placeholder gridMap
    // grid_map::GridMap grid_map;
    // grid_map::Length grid_map_dimensions(1.0, 1.0); // lengths in x,y directions [m]
    // double grid_map_resolution = 0.1; // resolution [m]
    // grid_map::Position grid_map_origin(0.0, 0.0); // origin [m]
    // grid_map.setGeometry(grid_map_dimensions, 
    //                         grid_map_resolution, 
    //                         grid_map_origin);
    // grid_map.add("elevation", 0.0); // add layer with value to initialize to everywhere'
    // grid_map.setFrameId("odom");

    // grid_map_msgs::msg::GridMap grid_map_msg;
    // grid_map_msg = *grid_map::GridMapRosConverter::toMessage(grid_map);
    terrain_msg.gridmap = *grid_map::GridMapRosConverter::toMessage(map);

    terrainPub_->publish(terrain_msg);

    visualizePlanarRegions(terrain_msg);
}


void Visualizer::overlayCenters(const cv::Mat & color_depth_image, const std::vector<std::vector<double>> & centers)
{
    // overlay center grid on color version of depth image
    cv::Mat overlaid_image = color_depth_image.clone();

    cv::Vec3b color(255, 0, 255);
    displayCenterGrid(overlaid_image, color, centers);

    center_grid_img_ptr_->header = fin_depth_img_ptr_->header;
    // center_grid_img_ptr_->header.stamp = ros::Time::now();
    center_grid_img_ptr_->encoding = sensor_msgs::image_encodings::BGR8;

    center_grid_img_ptr_->image = overlaid_image;
    center_grid_img_pub_.publish(center_grid_img_ptr_->toImageMsg());
}

void Visualizer::convertDepthImageToColor(cv::Mat & color_depth_image, const cv::Mat & depth_image)
{
    double min_depth = 0.0, max_depth = 0.0;
    cv::minMaxLoc(depth_image, &min_depth, &max_depth);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Converting to 8UC3...");

    for (int r = 0; r < color_depth_image.rows; r++)
    {
        for (int c = 0; c < color_depth_image.cols; c++)
        {
            float depth = depth_image.at<float>(r, c);

            if (std::isnan(depth) || std::abs(depth) < 1e-6)
            {
                continue;
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(), "   (r, c): (" << r << ", " << c << ")");
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       depth: " << depth);

            int quantized_depth = (int) (depth * 255.0 / max_depth); // just scaling by max depth in image. If we do full max depth than image is really hard to see.

            cv::Vec3b color = cv::Vec3b(quantized_depth, quantized_depth, quantized_depth);
            color_depth_image.at<cv::Vec3b>(r, c) = color;
        }
    }    
}

void Visualizer::displayCenterGrid(cv::Mat & image, const cv::Vec3b & color, const std::vector<std::vector<double>> & centers)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [SuperpixelColorSegmenter::displayCenterGrid]");
    
    // Display center grid
    for (int i = 0; i < (int) centers.size(); i++) 
    {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       center[" << i << "]: (" << centers[i][0] << ", " << centers[i][1] << ")");
        cv::circle(image, cv::Point(centers[i][0], centers[i][1]), 2, color, -1);
    }

    return;
}

void Visualizer::colorClusters(const cv::Mat & color_depth_image,
                                const cv::Mat & clusters)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [Visualizer::colorClusters]");
    // overlay center grid on color version of depth image
    cv::Mat color_cluster_image = color_depth_image.clone();

    // build ector of random colors for clusters
    // std::vector<cv::Scalar> colors(centers.size());
    // for (int i = 0; i < (int) colors.size(); i++)
    // {
    //     colors[i] = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
    // }

    // iterate through valid pixels and color
    for (int r = 0; r < color_depth_image.rows; r++)
    {
        for (int c = 0; c < color_depth_image.cols; c++)
        {    
            // if (isPixelValid(depth_image, label_image, normal_image, current)) 

            int cluster_id = clusters.at<int>(r, c);
            if (cluster_id != -1)
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(), "   (r, c): (" << r << ", " << c << ")");
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       cluster_id: " << cluster_id);
                cv::Scalar color = colors_[cluster_id];
                // RCLCPP_INFO_STREAM(node_->get_logger(), "       color: " << color);
                color_cluster_image.at<cv::Vec3b>(r, c) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }
    }
    
    colored_cluster_img_ptr_->header = fin_depth_img_ptr_->header;
    // colored_cluster_img_ptr_->header.stamp = ros::Time::now();
    colored_cluster_img_ptr_->encoding = sensor_msgs::image_encodings::BGR8;

    colored_cluster_img_ptr_->image = color_cluster_image;
    colored_cluster_img_pub_.publish(colored_cluster_img_ptr_->toImageMsg());

}

void Visualizer::colorClusterPointCloud(const cv::Mat & depth_image, const cv::Mat & clusters)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [Visualizer::colorClusterPointCloud]");

    // std::lock_guard<std::mutex> lock(cloud_mutex_);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "       Coloring cluster point cloud ...");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // colored_cloud->header = fin_depth_img_ptr_->header;
    // colored_cloud->header.stamp = ros::Time::now();
    colored_cloud->width = depth_image.cols;
    colored_cloud->height = depth_image.rows;
    // colored_cloud->is_dense = cloud_ptr_->is_dense;
    colored_cloud->points.resize(colored_cloud->width * colored_cloud->height);

    // iterate through valid pixels and color
    pcl::PointXYZRGBA point = pcl::PointXYZRGBA();
    cv::Point pixel = cv::Point(0, 0);
    cv::Vec3f egocanPt = cv::Vec3f(0.0, 0.0, 0.0);
    int cluster_id = -1;
    cv::Scalar color = cv::Scalar(114, 0, 189);
    int idx = 0;
    for (int r = 0; r < depth_image.rows; r++)
    {
        for (int c = 0; c < depth_image.cols; c++)
        {
            pixel = cv::Point(c, r);

            pixelToEgocanFrame(egocanPt, pixel, depth_image.at<float>(r, c), params_.k_c_, params_.h_);

            point.x = egocanPt[0];
            point.y = egocanPt[1];
            point.z = egocanPt[2];

            cluster_id = clusters.at<int>(r, c);
            if (cluster_id != -1)
            {
                // color = colors_[cluster_id];
                // point.r = color[2];
                // point.g = color[1];
                // point.b = color[0];
                point.r = color[0];
                point.g = color[1];
                point.b = color[2];
                point.a = 255;
            } else
            {
                point.a = 128;
            }

            idx = r * colored_cloud->width + c;
            colored_cloud->points[idx] = point;
        }
    }

    sensor_msgs::msg::PointCloud2 colored_cloud_msg;
    pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
    colored_cloud_msg.header = fin_depth_img_ptr_->header;

    colored_point_cloud_pub_->publish(colored_cloud_msg);

    return;
}

void Visualizer::colorCentroids(const std::vector<std::vector<double>> & centers,
                                const std::vector<int> & center_counts)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   [Visualizer::colorCentroids]");

    // clear prior markers
    visualization_msgs::msg::MarkerArray clear_marker_array;
    visualization_msgs::msg::Marker clearMarker;
    clearMarker.id = 0;
    clearMarker.ns =  "clear";
    clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_marker_array.markers.push_back(clearMarker);    
    colored_centroids_pub_->publish(clear_marker_array);

    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < centers.size(); i++)
    {
        if (center_counts[i] == 0)
        {
            continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header = fin_depth_img_ptr_->header;
        marker.ns = "superpixel_centroids";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        cv::Point center_pixel = cv::Point(centers[i][0], centers[i][1]);
        float center_depth = centers[i][2];
        cv::Vec3f center_normal = cv::Vec3f(centers[i][3], centers[i][4], centers[i][5]);

        cv::Vec3f centerEgocanPt;
        pixelToEgocanFrame(centerEgocanPt, center_pixel, center_depth, params_.k_c_, params_.h_);

        marker.points.resize(2);
        double scale = 0.1;
        geometry_msgs::msg::Point p1, p2;
        p1.x = centerEgocanPt[0];
        p1.y = centerEgocanPt[1];
        p1.z = centerEgocanPt[2];
        p2.x = p1.x + scale * center_normal[0];
        p2.y = p1.y + scale * center_normal[1];
        p2.z = p1.z + scale * center_normal[2];

        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Centroid " << i << ": (" << p1.x << ", " << p1.y << ", " << p1.z << ")");
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Normal " << i << ": (" << centers[i][4] << ", " << centers[i][5] << ", " << centers[i][6] << ")");

        marker.points[0] = p1;
        marker.points[1] = p2;

        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.0;

        marker.color.a = 1.0;
        cv::Scalar color = colors_[i];
        marker.color.r = color[2] / 255.0;
        marker.color.g = color[1] / 255.0;
        marker.color.b = color[0] / 255.0;

        marker_array.markers.push_back(marker);
    }

    colored_centroids_pub_->publish(marker_array);
}

void Visualizer::outputToDatFile(const cv_bridge::CvImagePtr & raw_depth_img_ptr,
                                    const std::vector<std::vector<Eigen::Vector2d>> & superpixel_projections)
{
    rclcpp::Time time = raw_depth_img_ptr->header.stamp;
    std::ofstream dat_file;
    std::string dat_file_path = ament_index_cpp::get_package_share_directory("superpixels") + "/data/" + std::to_string(time.seconds()) + "_" + std::to_string(time.nanoseconds()) + ".dat";
    dat_file.open(dat_file_path);

    for (size_t i = 0; i < superpixel_projections.size(); i++)
    {
        for (size_t j = 0; j < superpixel_projections[i].size(); j++)
        {
            dat_file << superpixel_projections[i][j][0] << " " << superpixel_projections[i][j][1] << " " << i << std::endl;
        }
        // dat_file << std::endl;
    }

    dat_file.close();
}

void Visualizer::visualizePlanarRegionBoundaries(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                                    const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions)
{
    int counter = 0;
    visualization_msgs::msg::MarkerArray planarRegionMarkerArray;

    for (int i = 0; i < (int) planarRegions.size(); i++)
    {
        convex_plane_decomposition::PlanarRegion region = planarRegions[i];

        rclcpp::Time timeStamp = node_->get_clock()->now();

        switched_model::ConvexTerrain convexTerrain = terrainPtr->getConvexTerrainAtPositionInWorld(region.transformPlaneToWorld.translation(),
            [](const Eigen::Vector3d&) { return 0.0; });

        // Add region marker
        visualization_msgs::msg::Marker regionMarker;
        regionMarker.header.frame_id = "odom";
        regionMarker.header.stamp = timeStamp;
        regionMarker.ns = "local_regions";
        regionMarker.id = i;

        std::vector<geometry_msgs::msg::Point> boundary;
        boundary.reserve(convexTerrain.boundary.size() + 1);
    
        for (const auto& point : convexTerrain.boundary) 
        {
            const auto& pointInWorldFrame = switched_model::positionInWorldFrameFromPositionInTerrain({point.x(), point.y(), 0.0}, convexTerrain.plane);
            geometry_msgs::msg::Point pointMsg;
            pointMsg.x = pointInWorldFrame.x();
            pointMsg.y = pointInWorldFrame.y();
            pointMsg.z = pointInWorldFrame.z();
            boundary.emplace_back(pointMsg);
        }
    
        // Close the polygon
        const auto& pointInWorldFrame = switched_model::positionInWorldFrameFromPositionInTerrain(
            {convexTerrain.boundary.front().x(), convexTerrain.boundary.front().y(), 0.0}, convexTerrain.plane);
        geometry_msgs::msg::Point pointMsg;
        pointMsg.x = pointInWorldFrame.x();
        pointMsg.y = pointInWorldFrame.y();
        pointMsg.z = pointInWorldFrame.z();
        boundary.emplace_back(pointMsg);

        // Headers
        regionMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        regionMarker.action = visualization_msgs::msg::Marker::ADD;
        regionMarker.scale.x = 0.02; // Line width
        regionMarker.color.a = 1.0; // Opacity
        regionMarker.color.r = 0.0; // Red
        regionMarker.color.g = 0.4470; // Green
        regionMarker.color.b = 0.7410; // Blue
        regionMarker.points = boundary;
        regionMarker.pose.orientation.w = 1.0; // No rotation
        regionMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
        planarRegionMarkerArray.markers.emplace_back(regionMarker);
    }

    if (planarRegions.size() < priorPlanarRegionsSize)
    {
        // Clear extra markers from prior visualization
        for (size_t j = planarRegions.size(); j < priorPlanarRegionsSize; j++)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );

            rclcpp::Time timeStamp = node_->get_clock()->now();

            // Add region marker
            visualization_msgs::msg::Marker regionMarker;
            regionMarker.header.frame_id = "odom";
            regionMarker.header.stamp = timeStamp;
            regionMarker.ns = "local_regions";
            regionMarker.id = j;
            regionMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            regionMarker.action = visualization_msgs::msg::Marker::DELETE;
            regionMarker.pose.position.x = 0.0;
            regionMarker.pose.position.y = 0.0;
            regionMarker.pose.position.z = 0.0;
            regionMarker.pose.orientation.w = 1.0; // No rotation
            regionMarker.text = ""; // Region ID as text
            // regionMarker.scale.x = 1.0; // radius
            // regionMarker.scale.y = 1.0; // radius
            regionMarker.scale.z = 0.05; // radius
            regionMarker.color.a = 1.0; // transparency
            regionMarker.color.r = 0.0; // red
            regionMarker.color.g = 0.0; // green
            regionMarker.color.b = 0.0; // blue
            regionMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
            planarRegionMarkerArray.markers.push_back(regionMarker);
        }
    }

    priorPlanarRegionsSize = planarRegions.size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing planar region markers of size " << planarRegionMarkerArray.markers.size());
    localRegionPublisher_->publish(planarRegionMarkerArray);
}

void Visualizer::visualizePlanarRegionNormals(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                                const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions)
{
    int counter = 0;
    visualization_msgs::msg::MarkerArray planarRegionNormalMarkerArray;

    for (int i = 0; i < (int) planarRegions.size(); i++)
    {
        convex_plane_decomposition::PlanarRegion region = planarRegions[i];

        rclcpp::Time timeStamp = node_->get_clock()->now();

        switched_model::ConvexTerrain convexTerrain = terrainPtr->getConvexTerrainAtPositionInWorld(region.transformPlaneToWorld.translation(),
            [](const Eigen::Vector3d&) { return 0.0; });

        // Add region ID marker
        visualization_msgs::msg::Marker regionNormalMarker;
        regionNormalMarker.header.frame_id = "odom";
        regionNormalMarker.header.stamp = timeStamp;
        regionNormalMarker.ns = "local_region_normals";
        regionNormalMarker.id = i;
        regionNormalMarker.type = visualization_msgs::msg::Marker::ARROW;
        regionNormalMarker.action = visualization_msgs::msg::Marker::ADD;
        regionNormalMarker.scale.x = 0.01;
        regionNormalMarker.scale.y = 0.02;
        regionNormalMarker.scale.z = 0.06;
        regionNormalMarker.points.reserve(2);
        double normalLength = 0.1;
        const Eigen::Vector3d surfaceNormal = normalLength * surfaceNormalInWorld(convexTerrain.plane);
        const Eigen::Vector3d startPointVec = convexTerrain.plane.positionInWorld;
        geometry_msgs::msg::Point startPoint;
        startPoint.x = startPointVec.x();
        startPoint.y = startPointVec.y();
        startPoint.z = startPointVec.z();
        geometry_msgs::msg::Point endPoint;
        endPoint.x = startPointVec.x() + surfaceNormal.x();
        endPoint.y = startPointVec.y() + surfaceNormal.y();
        endPoint.z = startPointVec.z() + surfaceNormal.z();
        regionNormalMarker.points.push_back(startPoint);
        regionNormalMarker.points.push_back(endPoint);
        regionNormalMarker.color.a = 1.0; // transparency
        regionNormalMarker.color.r = 0.0; // red
        regionNormalMarker.color.g = 0.4470; // green
        regionNormalMarker.color.b = 0.7410; // blue
        regionNormalMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
        regionNormalMarker.pose.orientation.w = 1.0; // No rotation
        planarRegionNormalMarkerArray.markers.push_back(regionNormalMarker);
    }

    if (planarRegions.size() < priorPlanarRegionsIDSize)
    {
        // Clear extra markers from prior visualization
        for (size_t j = planarRegions.size(); j < priorPlanarRegionsIDSize; j++)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );

            rclcpp::Time timeStamp = node_->get_clock()->now();

            // Add region ID marker
            visualization_msgs::msg::Marker regionNormalMarker;
            regionNormalMarker.header.frame_id = "odom";
            regionNormalMarker.header.stamp = timeStamp;
            regionNormalMarker.ns = "local_region_normals";
            regionNormalMarker.id = j;
            regionNormalMarker.type = visualization_msgs::msg::Marker::ARROW;
            regionNormalMarker.action = visualization_msgs::msg::Marker::DELETE;
            regionNormalMarker.pose.position.x = 0.0;
            regionNormalMarker.pose.position.y = 0.0;
            regionNormalMarker.pose.position.z = 0.0;
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );
            regionNormalMarker.pose.orientation.w = 1.0; // No rotation
            regionNormalMarker.text = ""; // Region ID as text
            // regionNormalMarker.scale.x = 1.0; // radius
            // regionNormalMarker.scale.y = 1.0; // radius
            regionNormalMarker.scale.z = 0.05; // radius
            regionNormalMarker.color.a = 1.0; // transparency
            regionNormalMarker.color.r = 0.0; // red
            regionNormalMarker.color.g = 0.0; // green
            regionNormalMarker.color.b = 0.0; // blue
            regionNormalMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
            planarRegionNormalMarkerArray.markers.push_back(regionNormalMarker);
        }
    }

    priorPlanarRegionsNormalSize = planarRegions.size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing planar region normal markers of size " << planarRegionNormalMarkerArray.markers.size());
    localRegionNormalPublisher_->publish(planarRegionNormalMarkerArray);
}

void Visualizer::visualizePlanarRegionIDs(const std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> & terrainPtr,
                                            const std::vector<convex_plane_decomposition::PlanarRegion> & planarRegions)
{
    int counter = 0;
    visualization_msgs::msg::MarkerArray planarRegionIDMarkerArray;

    for (int i = 0; i < (int) planarRegions.size(); i++)
    {
        convex_plane_decomposition::PlanarRegion region = planarRegions[i];

        rclcpp::Time timeStamp = node_->get_clock()->now();

        switched_model::ConvexTerrain convexTerrain = terrainPtr->getConvexTerrainAtPositionInWorld(region.transformPlaneToWorld.translation(),
            [](const Eigen::Vector3d&) { return 0.0; });

        // Add region ID marker
        visualization_msgs::msg::Marker regionIDMarker;
        regionIDMarker.header.frame_id = "odom";
        regionIDMarker.header.stamp = timeStamp;
        regionIDMarker.ns = "local_region_ids";
        regionIDMarker.id = i;
        regionIDMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        regionIDMarker.action = visualization_msgs::msg::Marker::ADD;
        regionIDMarker.pose.position.x = convexTerrain.plane.positionInWorld.x();
        regionIDMarker.pose.position.y = convexTerrain.plane.positionInWorld.y();
        regionIDMarker.pose.position.z = convexTerrain.plane.positionInWorld.z();
        // RCLCPP_INFO_STREAM(node_->get_logger(), "       Region " << i << " position: (" << regionIDMarker.pose.position.x << ", " << regionIDMarker.pose.position.y << ", " << regionIDMarker.pose.position.z << ")");
        regionIDMarker.pose.orientation.w = 1.0; // No rotation
        regionIDMarker.text = std::to_string(i); // Region ID as text
        // regionIDMarker.scale.x = 1.0; // radius
        // regionIDMarker.scale.y = 1.0; // radius
        regionIDMarker.scale.z = 0.05; // radius
        regionIDMarker.color.a = 1.0; // transparency
        regionIDMarker.color.r = 0.0; // red
        regionIDMarker.color.g = 0.0; // green
        regionIDMarker.color.b = 0.0; // blue
        regionIDMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
        planarRegionIDMarkerArray.markers.push_back(regionIDMarker);
    }

    if (planarRegions.size() < priorPlanarRegionsIDSize)
    {
        // Clear extra markers from prior visualization
        for (size_t j = planarRegions.size(); j < priorPlanarRegionsIDSize; j++)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );

            rclcpp::Time timeStamp = node_->get_clock()->now();

            // Add region ID marker
            visualization_msgs::msg::Marker regionIDMarker;
            regionIDMarker.header.frame_id = "odom";
            regionIDMarker.header.stamp = timeStamp;
            regionIDMarker.ns = "local_region_ids";
            regionIDMarker.id = j;
            regionIDMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            regionIDMarker.action = visualization_msgs::msg::Marker::DELETE;
            regionIDMarker.pose.position.x = 0.0;
            regionIDMarker.pose.position.y = 0.0;
            regionIDMarker.pose.position.z = 0.0;
            // RCLCPP_INFO_STREAM(node_->get_logger(), "       Filler region " << j );
            regionIDMarker.pose.orientation.w = 1.0; // No rotation
            regionIDMarker.text = ""; // Region ID as text
            // regionIDMarker.scale.x = 1.0; // radius
            // regionIDMarker.scale.y = 1.0; // radius
            regionIDMarker.scale.z = 0.05; // radius
            regionIDMarker.color.a = 1.0; // transparency
            regionIDMarker.color.r = 0.0; // red
            regionIDMarker.color.g = 0.0; // green
            regionIDMarker.color.b = 0.0; // blue
            regionIDMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // Lifetime of the marker
            planarRegionIDMarkerArray.markers.push_back(regionIDMarker);
        }
    }

    priorPlanarRegionsIDSize = planarRegions.size();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing planar region ID markers of size " << planarRegionIDMarkerArray.markers.size());
    localRegionIDPublisher_->publish(planarRegionIDMarkerArray);    
}

void Visualizer::visualizePlanarRegions(const convex_plane_decomposition_msgs::msg::PlanarTerrain & terrain_msg)
{
    std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> terrainPtr = std::make_unique<switched_model::SegmentedPlanesTerrainModel>(convex_plane_decomposition::fromMessage(terrain_msg));

    std::vector<convex_plane_decomposition::PlanarRegion> planarRegions = terrainPtr->planarTerrain().planarRegions;

    visualizePlanarRegionBoundaries(terrainPtr, planarRegions);
    visualizePlanarRegionNormals(terrainPtr, planarRegions);
    visualizePlanarRegionIDs(terrainPtr, planarRegions);
}
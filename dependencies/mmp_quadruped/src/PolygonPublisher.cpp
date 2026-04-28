#include <mmp_quadruped/PolygonPublisher.h>


namespace mmp {

PolygonPublisher::PolygonPublisher(const rclcpp::Node::SharedPtr& node, 
                                    const std::string & jsonFile)
{
  // std::cout << "[PolygonPublisher()]" << std::endl;
  jsonFile_ = jsonFile;
  // nodeHandle_ = nodeHandle;
  node_ = node;

  // observation subscriber
  std::string mpc_topic_prefix = "go2";
  auto observation_callback = 
      [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) 
      {
        // std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
        latest_x = latestObservation_.state;
        readyToPublish_ = true;
      };
  observationSub_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(mpc_topic_prefix
      + "_mpc_observation", 1, observation_callback);

  // Publish the rebar using cubes
  region_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("polygon_visualization", 1);

  std::ifstream json_fstream(jsonFile_);
  json_fstream >> json_file;
  // std::cout << "[PolygonPublisher() done]" << std::endl;
}

void PolygonPublisher::publishLocalPolygons()
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (!json_file["stone_setup"].getMemberNames().empty())
    marker_array = generateSteppingStoneArrayFromFile(jsonFile_);
  // else
  //   throw std::runtime_error("[PolygonPublisher::publishLocalPolygons()]: The stepping stones are not specified correctly");
  
  region_pub->publish(marker_array);

}

// ASSUMPTION: all stepping stones are flat and planar
visualization_msgs::msg::MarkerArray PolygonPublisher::generateSteppingStoneArrayFromFile(const std::string& setup_path) 
{
  visualization_msgs::msg::MarkerArray stoneMarkerArray;

  if (!readyToPublish_)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "  not ready to publish");
    return stoneMarkerArray;
  }

  // RCLCPP_INFO_STREAM(node_->get_logger(), "  [PolygonPublisher::generateSteppingStoneArrayFromFile]");
  Json::Value stone_file;
  std::ifstream stone_fstream(setup_path);
  stone_fstream >> stone_file;
  auto stone_setup = stone_file["stone_setup"];
  
  // if (stone_setup.getMemberNames().empty())
  //   throw std::runtime_error("[Stepping Stone visualizer] The stepping stones are not specified correctly");

  visualization_msgs::msg::Marker stoneMarker;
  stoneMarker.header.frame_id = "odom";

  stoneMarker.type = visualization_msgs::msg::Marker::CUBE;
  stoneMarker.action = visualization_msgs::msg::Marker::ADD;

  int counter = 0;
  int id = 0;
  for (auto s : stone_setup["poses"]) 
  {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "id: " << id);

    auto pose = stone_setup["poses"][counter];
    auto scale = stone_setup["scales"][counter];
    auto color = stone_setup["colors"][counter];

    // vector3_t bot_left_pt, bot_right_pt, top_right_pt;
    // bot_left_pt << s[0][0].asDouble(), s[0][1].asDouble(), s[0][2].asDouble(); // 0.05;
    // bot_right_pt << s[1][0].asDouble(), s[1][1].asDouble(), s[1][2].asDouble(); // 0.05;
    // top_right_pt << s[2][0].asDouble(), s[2][1].asDouble(), s[2][2].asDouble(); // 0.05;
    // double height = 0.10; // defined in yaml, just grabbing from there

    // RCLCPP_INFO_STREAM(node_->get_logger(), "   bot_left_pt: " << bot_left_pt.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   bot_right_pt: " << bot_right_pt.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   top_right_pt: " << top_right_pt.transpose());

    // vector3_t stone_center_pt = (bot_left_pt + top_right_pt) / 2.;

    // std::cout << "    latest_q: " << latest_q.transpose() << std::endl;

    vector3_t stone_center_pt(pose[0].asDouble(), pose[1].asDouble(), pose[2].asDouble());
    vector3_t stone_scale(scale[0].asDouble(), scale[1].asDouble(), scale[2].asDouble());
    vector3_t stone_rpy(pose[3].asDouble(), pose[4].asDouble(), pose[5].asDouble()); // roll, pitch, yaw

    base_coordinate_t torso_pose = getBasePose(latest_x); // .head(6);
    vector3_t torso_position = extractTorsoPosition(torso_pose);

    double stone_dist = (torso_position - stone_center_pt).norm();

    stoneMarker.id = id;

    // vector3_t widthVector = bot_right_pt - bot_left_pt;
    // vector3_t lengthVector = top_right_pt - bot_right_pt;

    // vector3_t cross =  (widthVector).cross(lengthVector);

    // vector3_t stone_normal = cross / cross.norm();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "   widthVector: " << widthVector.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   lengthVector: " << lengthVector.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   stone_normal: " << stone_normal.transpose());

    // vector3_t s_ray = bot_right_pt - stone_top_left_pt;


    // RCLCPP_INFO_STREAM(node_->get_logger(), "   cross: " << cross.transpose());

    // vector3_t e0 = widthVector.normalized();

    // vector3_t e1 = lengthVector.normalized();

    // vector3_t testVec(1.0, 1.0, 0.0);

    // if (e0.dot(testVec) < 0)
    // {
    //   e0 = -e0;
    //   e1 = -e1;
    // }

    // double width = widthVector.norm();
    // double length = lengthVector.norm();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "   s_mid: " << s_mid.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   e0: " << e0.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   width: " << width);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   e1: " << e1.transpose());
    // RCLCPP_INFO_STREAM(node_->get_logger(), "   length: " << length);

    // vector3_t mid_pt = (bot_left_pt + top_right_pt) / 2.0;
    // vector3_t stone_center = mid_pt - (height / 2.0) * stone_normal;

    stoneMarker.pose.position.x = stone_center_pt[0]; // - (sN[0] * 0.025); // so stepping on top of stone
    stoneMarker.pose.position.y = stone_center_pt[1]; // - (sN[1] * 0.025);
    stoneMarker.pose.position.z = stone_center_pt[2]; // - (sN[2] * 0.025);

    stoneMarker.scale.x = stone_scale[0]; // width
    stoneMarker.scale.y = stone_scale[1]; // length
    stoneMarker.scale.z = stone_scale[2]; // height

    // vector3_t orientation = calculateOrientationFromUnitNorms(e0, e1);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "    yaw: " << yaw);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    pitch: " << pitch);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "    roll: " << roll);

    tf2::Quaternion q_stone;
    q_stone.setRPY(stone_rpy[0], stone_rpy[1], stone_rpy[2]); // roll, pitch, yaw

    // RCLCPP_INFO_STREAM(node_->get_logger(), "   yaw: " << yaw);

    // RCLCPP_INFO_STREAM(node_->get_logger(), "   q_stone: " << q_stone.x() << ", " << q_stone.y() << ", " << q_stone.z() << ", " << q_stone.w());

    stoneMarker.pose.orientation.x = q_stone.x();
    stoneMarker.pose.orientation.y = q_stone.y();
    stoneMarker.pose.orientation.z = q_stone.z();
    stoneMarker.pose.orientation.w = q_stone.w();

    stoneMarker.color.a = 1.0;
    stoneMarker.color.r = color[0].asDouble();
    stoneMarker.color.g = color[1].asDouble();
    stoneMarker.color.b = color[2].asDouble();

    stoneMarkerArray.markers.push_back(stoneMarker);

    id++;
    counter++;
  }

  // std::cout << "number of local regions: " << id << std::endl;


  return stoneMarkerArray;
}

}  // namespace mmp

#ifndef MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H
#define MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H

// #include <ros/time.h>
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>

namespace message_filters
{

class ClockResetMonitor
{
public:
  ClockResetMonitor(rclcpp::Duration threshold=rclcpp::Duration::from_seconds(1)):
    threshold_(threshold)
    {}
    
  void setNode(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
    last_clock_time_ = node_->get_clock()->now(); // rclcpp::Time();
  }

  bool update()
  {
    rclcpp::Time cur_time = node_->get_clock()->now(); // rclcpp::Time();

    bool res = (last_clock_time_ - cur_time) > threshold_;
    
    if (res)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Resetting detector: New clock time (" << cur_time.nanoseconds() << ") is older than previous value (" << last_clock_time_.nanoseconds() << ") by more greater than [" << threshold_.seconds() << "]");
      cur_time = node_->get_clock()->now(); // rclcpp::Time();
    }
    last_clock_time_ = cur_time;

    return res;
  }

private:
  // ros::Time last_clock_time_;
  // ros::Duration threshold_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Duration threshold_;
  rclcpp::Time last_clock_time_;
};

}

#endif //MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H

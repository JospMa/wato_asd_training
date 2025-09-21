#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  using PathMsg = nav_msgs::msg::Path;
  using OdomMsg = nav_msgs::msg::Odometry;
  using TwistMsg = geometry_msgs::msg::Twist;

  robot::ControlCore core_;

  rclcpp::Subscription<PathMsg>::SharedPtr sub_path_;
  rclcpp::Subscription<OdomMsg>::SharedPtr sub_odom_;
  rclcpp::Publisher<TwistMsg>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  void onPath(const PathMsg::SharedPtr msg);
  void onOdom(const OdomMsg::SharedPtr msg);
  void onTick(); 
};

#endif 

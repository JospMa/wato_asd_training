#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace robot {

class ControlCore {
public:
  explicit ControlCore(const rclcpp::Logger& logger);

  // Inputs from the node (store latest)
  void processPath(const nav_msgs::msg::Path::SharedPtr msg);
  void processOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  [[nodiscard]] geometry_msgs::msg::Twist generateCommand();

private:
  // Aliases
  using Path = nav_msgs::msg::Path;
  using Odometry = nav_msgs::msg::Odometry;
  using Twist = geometry_msgs::msg::Twist;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Quaternion = geometry_msgs::msg::Quaternion;

  rclcpp::Logger logger_;
  Path::SharedPtr path_{};
  Odometry::SharedPtr odom_{};

  double lookahead_m_{0.8};     // how far ahead to track along the path
  double max_speed_mps_{0.7};   // cap forward speed

  std::optional<PoseStamped> findLookaheadPoint(double rx, double ry) const;
  double computeSteeringAngle(double rx, double ry, const PoseStamped& target) const;
  static double distance(double x1, double y1, double x2, double y2);
  static double yawFromQuat(const Quaternion& q);
};

} 

#endif 
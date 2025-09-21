#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot {

namespace {
constexpr double kPi = 3.141592653589793;
constexpr double kPosTol = 0.30; // stop if closer than this. quits the spinning
constexpr double kAngDeadband = 0.06; // dont rotate for tiny errors
constexpr double kSlowRadius = 0.80; // taper speeds inside this range to goal [m]

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}
//wrap angle to [-pi,pi]
inline double normAngle(double a) {
  while (a > kPi)  a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}
} 

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger), lookahead_m_(0.8), max_speed_mps_(0.7) {}

void ControlCore::processPath(const Path::SharedPtr msg) { path_ = msg; }
void ControlCore::processOdometry(const Odometry::SharedPtr msg) { odom_ = msg; }

geometry_msgs::msg::Twist ControlCore::generateCommand() {
  Twist cmd; 

  //need path and odom 
  if (!path_ || !odom_ || path_->poses.empty()) return cmd;

  const auto& P  = odom_->pose.pose.position;
  const double rx = P.x;
  const double ry = P.y;

  // stop at goal
  const auto& goalP = path_->poses.back().pose.position;
  const double gdx = goalP.x - rx;
  const double gdy = goalP.y - ry;
  const double goal_dist = std::hypot(gdx, gdy);
  if (goal_dist < kPosTol) {
    return Twist{};
  }

  auto maybe_target = findLookaheadPoint(rx, ry);
  if (!maybe_target) {
    return cmd; 
  }

  double steer = computeSteeringAngle(rx, ry, *maybe_target);

  // dont wiggle for tiny errors
  if (std::abs(steer) < kAngDeadband) steer = 0.0;

  // slow down as getting there
  double slow_gain = 1.0;
  if (goal_dist < kSlowRadius) {
    slow_gain = clamp(goal_dist / kSlowRadius, 0.25, 1.0); // taper but never zero
  }

  const double k_ang = 1.8; // turn aggressiveness
  cmd.angular.z = clamp(k_ang * steer * slow_gain, -1.5, 1.5);

  // Linear cmd: scale down when turning hard
  const double err = std::abs(steer);
  const double heading_scale = std::cos(std::min(err, 1.2));    
  const double v = max_speed_mps_ * clamp(heading_scale, 0.2, 1.0) * slow_gain;
  cmd.linear.x = clamp(v, 0.0, max_speed_mps_);

  return cmd;
}

std::optional<ControlCore::PoseStamped>
ControlCore::findLookaheadPoint(double rx, double ry) const {
  if (!path_ || path_->poses.size() < 2) return std::nullopt;

  // find nearest index to robot
  size_t nearest_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_->poses.size(); ++i) {
    const auto& pp = path_->poses[i].pose.position;
    const double dx = pp.x - rx, dy = pp.y - ry;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) { best_d2 = d2; nearest_idx = i; }
  }

  // walk forward accumulating length until we reach lookahead_m_
  const double Ld = lookahead_m_;
  double acc = 0.0;

  for (size_t i = nearest_idx; i + 1 < path_->poses.size(); ++i) {
    const auto& A = path_->poses[i].pose.position;
    const auto& B = path_->poses[i + 1].pose.position;
    const double seg = std::hypot(B.x - A.x, B.y - A.y);

    if (acc + seg >= Ld) {
      const double t = (Ld - acc) / std::max(seg, 1e-6);
      PoseStamped out = path_->poses[i]; 
      out.pose.position.x = A.x + t * (B.x - A.x);
      out.pose.position.y = A.y + t * (B.y - A.y);
      return out;
    }
    acc += seg;
  }

  return path_->poses.back();
}

double ControlCore::computeSteeringAngle(double rx, double ry, const PoseStamped& target) const {
  const double angle_to = std::atan2(target.pose.position.y - ry, target.pose.position.x - rx);
  const double yaw = yawFromQuat(odom_->pose.pose.orientation);
  return normAngle(angle_to - yaw);
}

double ControlCore::distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

double ControlCore::yawFromQuat(const Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

} 

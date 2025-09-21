#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  //  Aliases
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using Path = nav_msgs::msg::Path;
  using Odometry = nav_msgs::msg::Odometry;
  using PointStamped = geometry_msgs::msg::PointStamped;
  using Pose = geometry_msgs::msg::Pose;

  robot::PlannerCore core_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<PointStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_loop_;

  void onTick();
  void onMap(const OccupancyGrid::SharedPtr msg);
  void onOdom(const Odometry::SharedPtr msg);
  void onGoal(const PointStamped::SharedPtr msg);

  enum class State { IdleWaitingGoal, TrackingToGoal };
  State state_{State::IdleWaitingGoal};

  OccupancyGrid map_{};
  PointStamped goal_point_{};
  Pose robot_pose_{};
  bool has_goal_{false};
  bool isGoalReached() const;
  void computePlan();
};

#endif 

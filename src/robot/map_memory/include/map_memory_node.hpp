#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore core_;

  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using Odometry = nav_msgs::msg::Odometry;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub_costmap_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::TimerBase::SharedPtr timer_loop_;

  Odometry::SharedPtr odom_latest_;

  void onCostmap(const OccupancyGrid::SharedPtr& msg);
  void onOdom(const Odometry::SharedPtr& msg);
  void onTick(); 
  nav_msgs::msg::Odometry::SharedPtr odom_data_;

};

#endif 

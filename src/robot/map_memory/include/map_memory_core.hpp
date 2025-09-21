#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot {

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Odometry = nav_msgs::msg::Odometry;

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  void updateCostmap(const OccupancyGrid& costmap);
  void updateOdometry(const Odometry& odom);

  [[nodiscard]] std::optional<OccupancyGrid>generateGlobalMap(const Odometry& odom);

private:
  rclcpp::Logger logger_;

  OccupancyGrid global_map_{};     
  OccupancyGrid latest_costmap_{};  

  double last_pos_x_{0.0};
  double last_pos_y_{0.0};
  const double move_threshold_m_{1.5}; 

  
  bool needs_merge_{false};  
  bool has_costmap_{false};    
  bool global_map_ready_{false};  

private:
  void initializeGlobalMap();                    
  void updateMap(const Odometry& odom);     
  bool isWithinBounds(int grid_x, int grid_y) const noexcept; 
};

} 

#endif 
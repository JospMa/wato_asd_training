#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot {

struct GridCoord {
  int x{0}, y{0};
  constexpr GridCoord() = default;
  constexpr GridCoord(int xx, int yy) : x(xx), y(yy) {}
  [[nodiscard]] constexpr bool operator==(const GridCoord& other) const noexcept { return x==other.x && y==other.y; }
  [[nodiscard]] constexpr bool operator!=(const GridCoord& other) const noexcept { return !(*this==other); }
};

struct GridCoordHash {
  std::size_t operator()(const GridCoord& idx) const noexcept {
    return std::hash<int>{}(idx.x) ^ (std::hash<int>{}(idx.y) << 1);
  }
};

struct AStarNode {
  GridCoord index{};
  double f_score{0.0};
  AStarNode() = default;
  AStarNode(const GridCoord& idx, double f) : index(idx), f_score(f) {}
};

struct CompareByF {
  bool operator()(const AStarNode& a, const AStarNode& b) const noexcept { return a.f_score > b.f_score; }
};

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger& logger);
  [[nodiscard]] nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::PointStamped& goal);

private:
  rclcpp::Logger logger_;
  [[nodiscard]] bool isValid(const GridCoord& cell, const nav_msgs::msg::OccupancyGrid& map) const;
  [[nodiscard]] double heuristic(const GridCoord& a, const GridCoord& b, const nav_msgs::msg::OccupancyGrid& map) const;

  void reconstructPath(const std::unordered_map<GridCoord, GridCoord, GridCoordHash>& came_from,
                       const GridCoord& current, const GridCoord& start_cell,
                       const nav_msgs::msg::OccupancyGrid& map, nav_msgs::msg::Path& path) const;
  
};

} 

#endif 

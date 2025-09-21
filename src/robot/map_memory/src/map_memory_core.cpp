#include "map_memory_core.hpp"
#include <cmath>
#include <optional>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger),
    last_pos_x_(0.0),
    last_pos_y_(0.0),
    move_threshold_m_(1.5),
    needs_merge_(false),
    has_costmap_(false),
    global_map_ready_(false) {}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap) {
  latest_costmap_ = costmap;
  has_costmap_ = true;
}

//merge when moved far enough
void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry& odom) {
  const double cur_x = odom.pose.pose.position.x;
  const double cur_y = odom.pose.pose.position.y;
  const double dx = cur_x - last_pos_x_;
  const double dy = cur_y - last_pos_y_;
  const double threshold_sq = move_threshold_m_ * move_threshold_m_;

  //pythag 
  if (dx * dx + dy * dy >= threshold_sq) {
    last_pos_x_ = cur_x;
    last_pos_y_ = cur_y;
    needs_merge_ = true;
  }
}
//if not moved far or no costmap, nullopt, else if first, init map and then update map
std::optional<nav_msgs::msg::OccupancyGrid>
MapMemoryCore::generateGlobalMap(const nav_msgs::msg::Odometry& odom) {
  if (!needs_merge_ || !has_costmap_) return std::nullopt;

  if (!global_map_ready_) {
    initializeGlobalMap();
  } else {
    updateMap(odom);
  }
  needs_merge_ = false;
  return global_map_;
}

void MapMemoryCore::initializeGlobalMap() {
  const unsigned int global_width_cells = 300;
  const unsigned int global_height_cells = 300;

  auto& g = global_map_;
  g.header.frame_id = "sim_world";
  g.info.resolution = latest_costmap_.info.resolution;
  g.info.width = global_width_cells;
  g.info.height = global_height_cells;
  // Original centering
  g.info.origin.position.x = -static_cast<double>(global_width_cells) * g.info.resolution / 2.0;
  g.info.origin.position.y = -static_cast<double>(global_height_cells) * g.info.resolution / 2.0;
  g.info.origin.orientation.w = 1.0;

  // Start cells default to 0 
  g.data.assign(static_cast<size_t>(g.info.width) * g.info.height, 0);
  global_map_ready_ = true;
}

void MapMemoryCore::updateMap(const nav_msgs::msg::Odometry& odom) {
  //initialize a bunch of stuff to make later on stuff later
  // get yaw
  const double qx = odom.pose.pose.orientation.x;
  const double qy = odom.pose.pose.orientation.y;
  const double qz = odom.pose.pose.orientation.z;
  const double qw = odom.pose.pose.orientation.w;
  const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;

  //costmap geometry stuff
  const auto& cm_info = latest_costmap_.info;
  const double cm_res = cm_info.resolution;
  const double cm_org_x = cm_info.origin.position.x;
  const double cm_org_y = cm_info.origin.position.y;
  const unsigned int cm_w = cm_info.width;
  const unsigned int cm_h = cm_info.height;

  //global geometry stuff
  auto& g_data = global_map_.data;
  const auto& g_info = global_map_.info;
  const double gm_res = g_info.resolution;
  const double gm_org_x = g_info.origin.position.x;
  const double gm_org_y = g_info.origin.position.y;
  const unsigned int gm_w = g_info.width;
  const unsigned int gm_h = g_info.height;

  //now merge (gptd)
  size_t cm_idx = 0;
  for (unsigned int ly = 0; ly < cm_h; ++ly) {
    const double cy_m = cm_org_y + (static_cast<double>(ly) + 0.5) * cm_res;    //+.5 to put at center of cells
    for (unsigned int lx = 0; lx < cm_w; ++lx, ++cm_idx) {
      const double cx_m = cm_org_x + (static_cast<double>(lx) + 0.5) * cm_res;

      //gotta rotate the local and move it 
      const double gx_m = robot_x + (cx_m * cos_yaw - cy_m * sin_yaw);
      const double gy_m = robot_y + (cx_m * sin_yaw + cy_m * cos_yaw);
      //world meters to grid indices
      const int grid_x = static_cast<int>((gx_m - gm_org_x) / gm_res);
      const int grid_y = static_cast<int>((gy_m - gm_org_y) / gm_res);
      if (!isWithinBounds(grid_x, grid_y)) continue;

// Put this local cell into the global map at its spot if its "bigger" (occupied) so nothing breaks and deletes itself accidentally
      const size_t g_idx = static_cast<size_t>(grid_y) * gm_w + static_cast<unsigned int>(grid_x);
      int8_t& g_val = g_data[g_idx];
      const int incoming = static_cast<int>(latest_costmap_.data[cm_idx]);
      if (incoming > g_val) g_val = static_cast<int8_t>(incoming);
    }
  }
}

inline bool MapMemoryCore::isWithinBounds(int x, int y) const noexcept {
  return x >= 0 && y >= 0 &&
         static_cast<unsigned int>(x) < global_map_.info.width &&
         static_cast<unsigned int>(y) < global_map_.info.height;
}

} 

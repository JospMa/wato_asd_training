#include "costmap_core.hpp"

namespace robot
{
//    Change these values to tune costmap
CostmapCore::CostmapCore(const rclcpp::Logger& logger)
: logger_(logger),
  resolution_(0.4),  
  size_(120),      
  origin_x_(-24.0),  
  origin_y_(-24.0)   
{
    costmap_.resize(size_ * size_, -1); // initialize with unknown (-1)
}

    
//gptd this. make it better later. must finish assignment. lock in.
void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    // Reset map to free for this frame (keep your original behavior)
    std::fill(costmap_.begin(), costmap_.end(), 0);

    const int stride = 1; // set to 2 or 3 if you need a quick speed-up

    // Precompute trig incrementally
    const double angle0 = scan->angle_min;
    const double dtheta = scan->angle_increment;
    double c = std::cos(angle0);
    double s = std::sin(angle0);
    const double c_inc = std::cos(dtheta);
    const double s_inc = std::sin(dtheta);

    const float rmin = scan->range_min;
    const float rmax = scan->range_max;

    auto rot_step = [&]() {
        // rotate (c, s) by +dtheta without calling cos/sin again
        double c_new = c * c_inc - s * s_inc;
        double s_new = s * c_inc + c * s_inc;
        c = c_new; s = s_new;
    };

    for (size_t i = 0; i < scan->ranges.size(); i += stride) {
        float r = scan->ranges[i];
        if (!std::isfinite(r)) { rot_step(); continue; }        // skip NaN/inf
        if (r <= rmin || r >= rmax) { rot_step(); continue; }   // ignore out-of-range / max-range

        // endpoint in the lidar frame
        double x = static_cast<double>(r) * c;
        double y = static_cast<double>(r) * s;

        int gx, gy;
        if (convertToGrid(x, y, gx, gy)) {
            markObstacle(gx, gy);
        }

        rot_step();
    }

    inflateObstacles();
}

//gotta convert to grid
bool CostmapCore::convertToGrid(double x, double y, int &x_grid, int &y_grid) {
    // Translate world coordinates (x, y) into grid 
    x_grid = static_cast<int>((x - origin_x_) / resolution_);
    y_grid = static_cast<int>((y - origin_y_) / resolution_);

    // Check if inside bounds
    return (x_grid >= 0 && x_grid < size_ && 
            y_grid >= 0 && y_grid < size_);

    }
void CostmapCore::markObstacle(int x_grid, int y_grid) {
    costmap_[y_grid * size_ + x_grid] = 100;
}

void CostmapCore::inflateObstacles() {
    if (resolution_ <= 0.0 || size_ <= 0) return;

    // 1 source for inflation radius (meters). Tune later if buns
    const double inflation_radius_m = 1.3;
    const int    radius_cells = static_cast<int>(std::floor(inflation_radius_m / resolution_));
    if (radius_cells <= 0) return;

    std::vector<int8_t> out = costmap_;

    const double res2      = resolution_ * resolution_;
    const double radius2_m = inflation_radius_m * inflation_radius_m;

    // For each occupied cell, inflate neighbors within a circle
    for (int y = 0; y < size_; ++y) {
        for (int x = 0; x < size_; ++x) {
            if (costmap_[y * size_ + x] != 100) continue; //if not occupied skip
            //lowk dont know what is going on but basically if it hits a cell it inflates it and the cost decreases with distance linearly
            //clean ts up if time
            const int y_min = std::max(0,         y - radius_cells);
            const int y_max = std::min(size_ - 1, y + radius_cells);
            const int x_min = std::max(0,         x - radius_cells);
            const int x_max = std::min(size_ - 1, x + radius_cells);

            for (int ny = y_min; ny <= y_max; ++ny) {
                const int dy = ny - y;
                for (int nx = x_min; nx <= x_max; ++nx) {
                    const int dx = nx - x;

                    const double d2_m = (dx*dx + dy*dy) * res2;
                    if (d2_m > radius2_m) continue;

                    const double d_m  = std::sqrt(d2_m);      
                    const double frac = std::max(0.0, 1.0 - d_m / inflation_radius_m);
                    const int    cost = static_cast<int>(std::lround(100.0 * frac));

                    int8_t &cell = out[ny * size_ + nx];
                    if (cost > cell) cell = static_cast<int8_t>(cost); 
                }
            }
        }
    }

    costmap_.swap(out);
}



nav_msgs::msg::OccupancyGrid CostmapCore::generateOccupancyGrid(const rclcpp::Time& timestamp, const std::string& lidar_frame_id) {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = timestamp;
    grid_msg.header.frame_id = lidar_frame_id;

    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = size_;
    grid_msg.info.height = size_;
    grid_msg.info.origin.position.x = -size_ * resolution_ / 2; 
    grid_msg.info.origin.position.y = -size_ * resolution_ / 2; 
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data = costmap_;
    return grid_msg;
}

}
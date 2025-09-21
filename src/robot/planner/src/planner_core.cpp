#include "planner_core.hpp"

#include <array>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>

namespace robot
{


// Tuning cause robot kept stopping

namespace {
constexpr double OBSTACLE_WEIGHT = 4.5;  // stay further from inflated
constexpr int LETHAL_THRESHOLD = 100;  // please please please plssae do not go through walls
constexpr int UNKNOWN_COST = 80;   // treat unknown s risky
constexpr int OCCUPIED_THRESHOLD = 60;   // block diagonals near walls so it doesnt crash
constexpr bool ALLOW_DIAGONAL = true; // 8-connected grid

inline bool inBounds(int x, int y, int W, int H) {
  return (x >= 0 && y >= 0 && x < W && y < H);
}

// Prevent cutting the corner between two obstacles when moving diagonally
inline bool diagonalAllowed(const nav_msgs::msg::OccupancyGrid& m,
                            int x, int y, int nx, int ny) {
  if (!ALLOW_DIAGONAL) return (nx == x || ny == y);    
  const int dx = nx - x, dy = ny - y;
  if (dx == 0 || dy == 0) return true;      // straight moves always fine

  const int W = static_cast<int>(m.info.width);
  const int H = static_cast<int>(m.info.height);
  auto idx = [&](int cx, int cy){ return cy * W + cx; };

  const int sx = x + dx, sy = y;
  const int tx = x,      ty = y + dy;

  if (!inBounds(sx, sy, W, H) || !inBounds(tx, ty, W, H)) return false;

  const int8_t vs = m.data[static_cast<size_t>(idx(sx, sy))];
  const int8_t vt = m.data[static_cast<size_t>(idx(tx, ty))];

  auto passable = [](int8_t c){
    return (c < 0) || (c < OCCUPIED_THRESHOLD);
  };
  return passable(vs) && passable(vt);
}
} 



PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::PointStamped &goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = map.header.frame_id.empty() ? "map" : map.header.frame_id;

    // Convert world → grid
    const double res = map.info.resolution;
    const double ox  = map.info.origin.position.x;
    const double oy  = map.info.origin.position.y;

    auto toGrid = [&](double wx, double wy) -> GridCoord {
        return GridCoord(
            static_cast<int>((wx - ox) / res),
            static_cast<int>((wy - oy) / res)
        );
    };

    GridCoord start_cell = toGrid(start.position.x, start.position.y);
    GridCoord goal_cell  = toGrid(goal.point.x,  goal.point.y);

    const int W = static_cast<int>(map.info.width);
    const int H = static_cast<int>(map.info.height);
    if (!inBounds(start_cell.x, start_cell.y, W, H) ||
        !inBounds(goal_cell.x,  goal_cell.y,  W, H)) {
        return path;
    }

    // A* open set min-F priority 
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareByF> open_set;
    open_set.emplace(start_cell, heuristic(start_cell, goal_cell, map));

    std::unordered_map<GridCoord, double, GridCoordHash> g_score;
    g_score[start_cell] = 0.0;
    std::unordered_map<GridCoord, GridCoord, GridCoordHash> came_from;

    auto idx = [&](int gx, int gy){ return gy * W + gx; };
    auto costAt = [&](int gx, int gy) -> int8_t {
        if (!inBounds(gx, gy, W, H)) return static_cast<int8_t>(LETHAL_THRESHOLD);
        return map.data[static_cast<size_t>(idx(gx, gy))];
    };


    static const std::array<std::pair<int,int>,8> NBR = {{
        {+1,0},{-1,0},{0,+1},{0,-1},{+1,+1},{-1,-1},{+1,-1},{-1,+1}
    }};

    while (!open_set.empty())
    {
        AStarNode current = open_set.top();
        open_set.pop();

        if (current.index == goal_cell)
        {
            reconstructPath(came_from, current.index, start_cell, map, path);
            return path;
        }

        const int cx = current.index.x;
        const int cy = current.index.y;

        for (auto [dx,dy] : NBR)
        {
            const int nx = cx + dx;
            const int ny = cy + dy;
            GridCoord neighbor(nx, ny);

            if (!isValid(neighbor, map)) continue;

            // prevent diagonal cutting along walls
            if (!diagonalAllowed(map, cx, cy, nx, ny)) continue;

            // step length in grid
            const bool diag = (dx != 0 && dy != 0);
            const double step_cost = diag ? std::sqrt(2.0) : 1.0;

            // inflation-aware penalty
            int8_t c = costAt(nx, ny);                
            if (c >= LETHAL_THRESHOLD) continue;       // please plsaease plasea please dont sell me
            if (c < 0) c = static_cast<int8_t>(UNKNOWN_COST);
            const double cell_penalty = OBSTACLE_WEIGHT * (static_cast<double>(c) / 100.0);

            const double tentative_g = g_score[current.index] + step_cost + cell_penalty;

            auto it = g_score.find(neighbor);
            if (it == g_score.end() || tentative_g < it->second)
            {
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g;

                const double h = heuristic(neighbor, goal_cell, map)
                               + 1e-3 * (std::abs(nx - goal_cell.x) + std::abs(ny - goal_cell.y));

                open_set.emplace(neighbor, tentative_g + h);
            }
        }
    }


    return path;
}

bool PlannerCore::isValid(const GridCoord &cell, const nav_msgs::msg::OccupancyGrid &map) const
{
    const int W = static_cast<int>(map.info.width);
    const int H = static_cast<int>(map.info.height);
    if (!inBounds(cell.x, cell.y, W, H)) return false;

    const int idx = cell.y * W + cell.x;
    const int8_t c = map.data[static_cast<size_t>(idx)];
    return (c < 0) || (c < LETHAL_THRESHOLD);
}

double PlannerCore::heuristic(const GridCoord &a, const GridCoord &b, const nav_msgs::msg::OccupancyGrid &) const
{
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx*dx + dy*dy);
}

void PlannerCore::reconstructPath(
    const std::unordered_map<GridCoord, GridCoord, GridCoordHash> &came_from,
    const GridCoord &current,
    const GridCoord &start_cell,
    const nav_msgs::msg::OccupancyGrid &map,
    nav_msgs::msg::Path &path) const
{
    GridCoord cur = current;

    // world conversion
    const double res = map.info.resolution;
    const double ox  = map.info.origin.position.x;
    const double oy  = map.info.origin.position.y;

    while (came_from.find(cur) != came_from.end() || cur == start_cell)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map.header.frame_id.empty() ? "map" : map.header.frame_id;
        pose.pose.position.x = cur.x * res + ox;
        pose.pose.position.y = cur.y * res + oy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);

        if (cur == start_cell) break;
        cur = came_from.at(cur);
    }

    std::reverse(path.poses.begin(), path.poses.end());
}



} 

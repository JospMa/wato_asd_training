#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger()))
{

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        this->laserCallback(scan);
      });

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.processLaserScan(scan);
  auto costmap_msg = costmap_.generateOccupancyGrid(scan->header.stamp, scan->header.frame_id);
  costmap_pub_->publish(costmap_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
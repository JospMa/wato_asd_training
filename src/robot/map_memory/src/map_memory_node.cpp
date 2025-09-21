#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : rclcpp::Node("map_memory"),
    core_(robot::MapMemoryCore(this->get_logger()))
{
  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_costmap_ = this->create_subscription<OccupancyGrid>(
      "/costmap", sensor_qos,
      [this](const OccupancyGrid::SharedPtr msg) { onCostmap(msg); });
  sub_odom_ = this->create_subscription<Odometry>(
      "/odom/filtered", sensor_qos,
      [this](const Odometry::SharedPtr msg) { onOdom(msg); });

  auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  pub_map_ = this->create_publisher<OccupancyGrid>("/map", map_qos);

  // ask the core if a map update is ready and publish if so
  timer_loop_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { onTick(); });

  // Will be set once the first odom arrives.
  odom_latest_ = nullptr;
}

void MapMemoryNode::onCostmap(const OccupancyGrid::SharedPtr& msg) {
  core_.updateCostmap(*msg);
}

void MapMemoryNode::onOdom(const Odometry::SharedPtr& msg) {
  // Cache odom for timer and lets core do stuff if dist moved
  odom_latest_ = msg;
  core_.updateOdometry(*msg);
}

void MapMemoryNode::onTick() {
  if (!odom_latest_) return;

  auto maybe_global = core_.generateGlobalMap(*odom_latest_);
  if (maybe_global) {
    maybe_global->header.stamp = now();
    pub_map_->publish(std::move(*maybe_global));
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

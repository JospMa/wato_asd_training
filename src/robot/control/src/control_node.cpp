#include "control_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

ControlNode::ControlNode()
: rclcpp::Node("control_node"), core_(robot::ControlCore(this->get_logger()))
{
  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_path_ = create_subscription<PathMsg>(
      "/path", rclcpp::QoS(10),
      std::bind(&ControlNode::onPath, this, std::placeholders::_1));

  sub_odom_ = create_subscription<OdomMsg>(
      "/odom/filtered", sensor_qos,
      std::bind(&ControlNode::onOdom, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<TwistMsg>("/cmd_vel", rclcpp::QoS(10));

  timer_ = create_wall_timer(100ms, std::bind(&ControlNode::onTick, this));
}

void ControlNode::onPath(const PathMsg::SharedPtr msg) {
  core_.processPath(msg);
}

void ControlNode::onOdom(const OdomMsg::SharedPtr msg) {
  core_.processOdometry(msg);
}

void ControlNode::onTick() {
  auto cmd = core_.generateCommand();
  pub_cmd_->publish(cmd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

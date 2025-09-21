#include "planner_node.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

PlannerNode::PlannerNode()
: rclcpp::Node("planner_node"),
  core_(robot::PlannerCore(this->get_logger()))
{
 
  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_map_ = create_subscription<OccupancyGrid>(
      "/map", sensor_qos,
      std::bind(&PlannerNode::onMap, this, std::placeholders::_1));

  sub_goal_ = create_subscription<PointStamped>(
      "/goal_point", rclcpp::QoS(10),
      std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));

  sub_odom_ = create_subscription<Odometry>(
      "/odom/filtered", sensor_qos,
      std::bind(&PlannerNode::onOdom, this, std::placeholders::_1));

  auto latched = rclcpp::QoS(1).reliable().transient_local();
  pub_path_ = create_publisher<Path>("/path", latched);

  // Periodic replan/progress check
  timer_loop_ = create_wall_timer(500ms, std::bind(&PlannerNode::onTick, this));
}

void PlannerNode::onMap(const OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  if (state_ == State::TrackingToGoal) {
    computePlan();
  }
}

//set goal and plan to it
void PlannerNode::onGoal(const PointStamped::SharedPtr msg) {
  goal_point_ = *msg;
  has_goal_ = true;
  state_ = State::TrackingToGoal;
  computePlan();
}

void PlannerNode::onOdom(const Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::onTick() {
  if (state_ != State::TrackingToGoal) return;

  if (isGoalReached()) {
    state_ = State::IdleWaitingGoal;
    has_goal_ = false;
    return;
  }

  computePlan();
}
//robot kept spinning when it reached so we do this
bool PlannerNode::isGoalReached() const {
  const double dx = goal_point_.point.x - robot_pose_.position.x;
  const double dy = goal_point_.point.y - robot_pose_.position.y;
  return std::sqrt(dx*dx + dy*dy) < 0.5; // 0.5 m tolerance, edit if need more/less
}
//only plan if have goal and map
void PlannerNode::computePlan() {
  if (!has_goal_ || map_.data.empty()) {
    return;
  }

  auto path = core_.planPath(map_, robot_pose_, goal_point_);
  if (path.header.frame_id.empty()) path.header.frame_id = map_.header.frame_id;
  path.header.stamp = now();
  pub_path_->publish(path);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

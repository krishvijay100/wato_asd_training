#include <chrono>
#include <functional>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"),
      control_(this->get_logger()) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) return;

  auto cmd = control_.computeCommand(*current_path_, *robot_odom_);
  if (!cmd) return;

  cmd_vel_pub_->publish(*cmd);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

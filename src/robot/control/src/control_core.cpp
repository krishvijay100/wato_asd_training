#include "control_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeCommand(const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) const {
  if (path.poses.empty()) return std::nullopt;

  const auto& robot_pose = odom.pose.pose;

  //stop if within goal tolerance of path's final waypoint
  const auto& final_point = path.poses.back().pose.position;
  if (computeDistance(robot_pose.position, final_point) < goal_tolerance_) {
    geometry_msgs::msg::Twist stop;
    return stop;  //zero-initialized Twist = full stop
  }

  auto lookahead = findLookaheadPoint(path, robot_pose);
  if (!lookahead) return std::nullopt;

  return computeVelocity(*lookahead, robot_pose);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& robot_pose) const {
  if (path.poses.empty()) return std::nullopt;

  //scan from the start; pick first waypoint at least lookahead_distance_ away
  for (const auto& pose_stamped : path.poses) {
    const double d = computeDistance(robot_pose.position, pose_stamped.pose.position);
    if (d >= lookahead_distance_) {
      return pose_stamped;
    }
  }
  //fallback: no waypoint far enough, use the final one so the robot keeps chasing the goal instead of freezing short
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target, const geometry_msgs::msg::Pose& robot_pose) const {
  geometry_msgs::msg::Twist cmd;

  const double theta = extractYaw(robot_pose.orientation);

  //express lookahead in the robot's frame
  const double dx = target.pose.position.x - robot_pose.position.x;
  const double dy = target.pose.position.y - robot_pose.position.y;
  const double x_local =  dx * std::cos(theta) + dy * std::sin(theta);
  const double y_local = -dx * std::sin(theta) + dy * std::cos(theta);

  const double heading_error = std::atan2(y_local, x_local);

  double angular_z = angular_gain_ * heading_error;
  angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);

  cmd.linear.x = linear_speed_;
  cmd.angular.z = angular_z;
  return cmd;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}

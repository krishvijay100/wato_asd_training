#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    //compute the next Twist command from the current path and robot odometry.
    //returns std::nullopt when there is nothing to publish (no path / no odom yet).
    std::optional<geometry_msgs::msg::Twist> computeCommand(const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) const;

  private:
    //tuning parameters
    static constexpr double lookahead_distance_ = 1.0;
    static constexpr double goal_tolerance_ = 0.1;
    static constexpr double linear_speed_ = 0.5;
    static constexpr double angular_gain_ = 1.0;
    static constexpr double max_angular_speed_ = 1.0; //clamp

    rclcpp::Logger logger_;

    //pick a target waypoint on the path:
    //  -first waypoint whose distance from the robot is >= lookahead_distance_
    //  -if no such waypoint exists, return the path's final waypoint as a fallback
    //  -if the path is empty, return nullopt
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& robot_pose) const;

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target, const geometry_msgs::msg::Pose& robot_pose) const;

    //euclidean distance
    static double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

    static double extractYaw(const geometry_msgs::msg::Quaternion& q);
};

}

#endif

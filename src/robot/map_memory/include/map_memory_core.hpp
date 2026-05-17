#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    //store the latest local costmap (set the unmerged flag)
    void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    //update the cached robot pose from a new odometry message
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    //returns true if conditions for a merge are satisfied
    //(robot has moved >= distance_threshold_ since last update and a new costmap has arrived)
    bool shouldUpdate() const;

    //merge the latest local costmap into the global map, then mark the costmap consumed
    //and record the robot's current pose as the new "last update" pose.
    void aggregate();

    //expose the global map for the node to publish.
    const nav_msgs::msg::OccupancyGrid& globalMap() const { return global_map_; }

  private:
    //global map parameters
    static constexpr double resolution_ = 0.1;            //meters per cell
    static constexpr int global_width_ = 600;             //cells
    static constexpr int global_height_ = 600;            //cells
    static constexpr double global_origin_x_ = -30.0;     //meters
    static constexpr double global_origin_y_ = -30.0;     //meters
    static constexpr double distance_threshold_ = 1.5;    //meters

    //persistent global map (in world frame)
    nav_msgs::msg::OccupancyGrid global_map_;

    //latest local costmap (in robot frame), kept until consumed by aggregate()
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_updated_ = false;

    //robot pose (world frame), updated continuously from /odom/filtered
    double x_curr_ = 0.0;
    double y_curr_ = 0.0;
    double theta_curr_ = 0.0;

    //robot pose at the most recent successful map update
    double x_last_ = 0.0;
    double y_last_ = 0.0;

    rclcpp::Logger logger_;

    //extract yaw (rotation around vertical axis) from a quaternion
    static double extractYaw(const geometry_msgs::msg::Quaternion& q);

    //initialize the global map header / info / data on first use
    void initializeGlobalMap();
};

}

#endif

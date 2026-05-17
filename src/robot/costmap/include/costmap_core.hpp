#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot
{

class CostmapCore {
  public:

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    //convert a single laser scan into a fully-populated OccupancyGrid.
    nav_msgs::msg::OccupancyGrid processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    //costmap parameters
    static constexpr double resolution_ = 0.1;          //meters per cell
    static constexpr int width_ = 300;                  //cells
    static constexpr int height_ = 300;                 //cells
    static constexpr double inflation_radius_ = 1.0;    //meters
    static constexpr int8_t max_cost_ = 100;
    static constexpr int inflation_cells_ = static_cast<int>(inflation_radius_ / resolution_);

    //grid storage, grid_[y][x]
    std::vector<std::vector<int8_t>> grid_;

    rclcpp::Logger logger_;

    //helper methods
    void resetGrid();
    void markObstacle(int gx, int gy);
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid buildMessage(const std_msgs::msg::Header& scan_header) const;
};

}

#endif

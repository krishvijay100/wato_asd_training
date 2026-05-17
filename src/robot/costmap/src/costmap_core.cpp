#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : grid_(height_, std::vector<int8_t>(width_, 0)),
      logger_(logger) {}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  resetGrid();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double r = scan->ranges[i];
    if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) {
      continue;
    }

    const double angle = scan->angle_min + i * scan->angle_increment;
    const double x = r * std::cos(angle);
    const double y = r * std::sin(angle);

    const int gx = static_cast<int>(std::floor(x / resolution_)) + width_ / 2;
    const int gy = static_cast<int>(std::floor(y / resolution_)) + height_ / 2;

    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
      continue;
    }

    markObstacle(gx, gy);
  }

  inflateObstacles();
  return buildMessage(scan->header);
}

void CostmapCore::resetGrid() {
  for (auto& row : grid_) {
    std::fill(row.begin(), row.end(), 0);
  }
}

void CostmapCore::markObstacle(int gx, int gy) {
  grid_[gy][gx] = max_cost_;
}

void CostmapCore::inflateObstacles() {
  //snapshot of the "true obstacle" pattern so inflation doesn't feed on itself
  const auto source = grid_;

  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      if (source[gy][gx] != max_cost_) continue;

      for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
        for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
          const int nx = gx + dx;
          const int ny = gy + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

          const double distance_cells = std::sqrt(static_cast<double>(dx * dx + dy * dy));
          if (distance_cells > inflation_cells_) continue;

          const double distance_meters = distance_cells * resolution_;
          const int8_t cost = static_cast<int8_t>(max_cost_ * (1.0 - distance_meters / inflation_radius_));

          if (cost > grid_[ny][nx]) {
            grid_[ny][nx] = cost;
          }
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildMessage(const std_msgs::msg::Header& scan_header) const {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = scan_header.stamp;
  msg.header.frame_id = "robot/chassis/lidar";

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = -(width_ / 2.0) * resolution_;
  msg.info.origin.position.y = -(height_ / 2.0) * resolution_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(static_cast<size_t>(width_) * height_);
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      msg.data[gy * width_ + gx] = grid_[gy][gx];
    }
  }

  return msg;
}

}

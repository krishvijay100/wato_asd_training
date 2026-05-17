#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger) {
  initializeGlobalMap();
}

void MapMemoryCore::initializeGlobalMap() {
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = resolution_;
  global_map_.info.width = global_width_;
  global_map_.info.height = global_height_;
  global_map_.info.origin.position.x = global_origin_x_;
  global_map_.info.origin.position.y = global_origin_y_;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(static_cast<size_t>(global_width_) * global_height_, 0);
}

void MapMemoryCore::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryCore::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_curr_ = msg->pose.pose.position.x;
  y_curr_ = msg->pose.pose.position.y;
  theta_curr_ = extractYaw(msg->pose.pose.orientation);
}

bool MapMemoryCore::shouldUpdate() const {
  if (!costmap_updated_) return false;
  const double dx = x_curr_ - x_last_;
  const double dy = y_curr_ - y_last_;
  return std::sqrt(dx * dx + dy * dy) >= distance_threshold_;
}

void MapMemoryCore::aggregate() {
  const int local_width = latest_costmap_.info.width;
  const int local_height = latest_costmap_.info.height;
  const double local_res = latest_costmap_.info.resolution;

  const double cos_t = std::cos(theta_curr_);
  const double sin_t = std::sin(theta_curr_);

  for (int j = 0; j < local_height; ++j) {
    for (int i = 0; i < local_width; ++i) {
      //step 1: local cell index -> local-frame meters (centered on robot)
      const double x_local = (i - local_width / 2.0) * local_res;
      const double y_local = (j - local_height / 2.0) * local_res;

      //step 2: local-frame meters -> world-frame meters (rotate + translate)
      const double x_world = x_local * cos_t - y_local * sin_t + x_curr_;
      const double y_world = x_local * sin_t + y_local * cos_t + y_curr_;

      //step 3: world-frame meters -> global cell index
      const int gi = static_cast<int>(std::floor((x_world - global_origin_x_) / resolution_));
      const int gj = static_cast<int>(std::floor((y_world - global_origin_y_) / resolution_));

      if (gi < 0 || gi >= global_width_ || gj < 0 || gj >= global_height_) {
        continue;
      }

      const size_t local_index = static_cast<size_t>(j) * local_width + i;
      const size_t global_index = static_cast<size_t>(gj) * global_width_ + gi;

      const int8_t local_val = latest_costmap_.data[local_index];
      const int8_t global_val = global_map_.data[global_index];

      //max-merge: an obstacle once seen is never forgotten
      if (local_val > global_val) {
        global_map_.data[global_index] = local_val;
      }
    }
  }

  //update the "last update" pose and mark the costmap consumed
  x_last_ = x_curr_;
  y_last_ = y_curr_;
  costmap_updated_ = false;

  //stamp the global map for downstream consumers
  global_map_.header.stamp = latest_costmap_.header.stamp;
}

double MapMemoryCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}

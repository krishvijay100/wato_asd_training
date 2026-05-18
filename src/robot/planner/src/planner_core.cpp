#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

CellIndex PlannerCore::worldToCell(double x, double y, const nav_msgs::msg::OccupancyGrid& map) {
  const int gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  const int gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return CellIndex(gx, gy);
}

void PlannerCore::cellToWorld(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map, double& x, double& y) {
  //waypoint at the cell center, not its corner
  x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::PointStamped& goal) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  const int width = map.info.width;
  const int height = map.info.height;

  if (width <= 0 || height <= 0 || map.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan: map is empty.");
    return path;
  }

  const CellIndex start = worldToCell(start_pose.position.x, start_pose.position.y, map);
  const CellIndex target = worldToCell(goal.point.x, goal.point.y, map);

  auto inBounds = [&](const CellIndex& c) {
    return c.x >= 0 && c.x < width && c.y >= 0 && c.y < height;
  };
  auto isBlocked = [&](const CellIndex& c) {
    return map.data[c.y * width + c.x] > obstacle_threshold_;
  };

  if (!inBounds(start) || !inBounds(target)) {
    RCLCPP_WARN(logger_,
                "Cannot plan: start or goal outside map bounds.");
    return path;
  }
  if (isBlocked(target)) {
    RCLCPP_WARN(logger_, "Cannot plan: goal cell is blocked.");
    return path;
  }

  //a* state
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_set<CellIndex, CellIndexHash> closed;
  std::unordered_map<CellIndex, double, CellIndexHash> g_cost;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> parent;

  g_cost[start] = 0.0;
  open.emplace(start, heuristic(start, target));

  //8-connectivity neighbor offsets
  static const int dxs[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  static const int dys[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

  bool found = false;

  while (!open.empty()) {
    const AStarNode current = open.top();
    open.pop();

    //skip stale duplicate entries (a better path was found earlier)
    if (closed.count(current.index)) continue;
    closed.insert(current.index);

    if (current.index == target) {
      found = true;
      break;
    }

    for (int k = 0; k < 8; ++k) {
      const CellIndex neighbor(current.index.x + dxs[k], current.index.y + dys[k]);
      if (!inBounds(neighbor)) continue;
      if (isBlocked(neighbor)) continue;
      if (closed.count(neighbor)) continue;

      const double step = heuristic(current.index, neighbor);
      const double tentative_g = g_cost[current.index] + step;

      const auto it = g_cost.find(neighbor);
      const double best_g = (it == g_cost.end())
                                ? std::numeric_limits<double>::infinity()
                                : it->second;

      if (tentative_g < best_g) {
        g_cost[neighbor] = tentative_g;
        parent[neighbor] = current.index;
        const double f = tentative_g + heuristic(neighbor, target);
        open.emplace(neighbor, f);
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A*: no path found from (%d,%d) to (%d,%d).",
                start.x, start.y, target.x, target.y);
    return path;
  }

  return reconstructPath(parent, start, target, map);
}

nav_msgs::msg::Path PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& parent,
    const CellIndex& start,
    const CellIndex& goal,
    const nav_msgs::msg::OccupancyGrid& map) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  //walk parents from goal back to start
  std::vector<CellIndex> reverse_cells;
  CellIndex c = goal;
  reverse_cells.push_back(c);
  while (c != start) {
    auto it = parent.find(c);
    if (it == parent.end()) {
      RCLCPP_WARN(logger_, "Parent chain broken during reconstruction.");
      path.poses.clear();
      return path;
    }
    c = it->second;
    reverse_cells.push_back(c);
  }
  std::reverse(reverse_cells.begin(), reverse_cells.end());

  path.poses.reserve(reverse_cells.size());
  for (const auto& cell : reverse_cells) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = map.header.frame_id;
    double wx, wy;
    cellToWorld(cell, map, wx, wy);
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }

  return path;
}

}

#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

//2D grid index
struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return x != other.x || y != other.y; }
};

//hash function for CellIndex so it can be used in std::unordered_map / std::unordered_set
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

//structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

//comparator for the priority queue (min-heap by f_score)
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    //run A* on the supplied global map, returning a Path message in the map's frame.
    //if no path is found, the returned Path will have an empty poses array.
    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::PointStamped& goal) const;

  private:
    //treat any cell with cost strictly greater than this as blocked
    static constexpr int8_t obstacle_threshold_ = 80;

    rclcpp::Logger logger_;

    //heuristic: euclidean distance between two cells (in cells, not meters)
    static double heuristic(const CellIndex& a, const CellIndex& b);

    //convert world (x,y) meters to a cell index in the supplied map
    static CellIndex worldToCell(double x, double y, const nav_msgs::msg::OccupancyGrid& map);

    //convert a cell index back to world (x,y) meters at the cell center
    static void cellToWorld(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map, double& x, double& y);

    //walk the parent chain to build the path, then convert each cell to a PoseStamped
    nav_msgs::msg::Path reconstructPath(
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& parent,
        const CellIndex& start,
        const CellIndex& goal,
        const nav_msgs::msg::OccupancyGrid& map) const;
};

}

#endif

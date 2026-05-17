#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{
  // 2D grid index
  struct CellIndex
  {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const
    {
      return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const
    {
      return (x != other.x || y != other.y);
    }
  };

  // Hash function for CellIndex so it can be used in std::unordered_map
  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      // A simple hash combining x and y
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };

  // Structure representing a node in the A* open set
  struct AStarNode
  {
    CellIndex index;
    double f_score; // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  };

  // Comparator for the priority queue (min-heap by f_score)
  struct CompareF
  {
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
      // We want the node with the smallest f_score on top
      return a.f_score > b.f_score;
    }
  };

  class PlannerCore
  {
  public:
    explicit PlannerCore(const rclcpp::Logger &logger);

    void configure(int occupancy_threshold, double unknown_penalty);

    // returns an empty path if no route exists.
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid &map,
                                 double start_x, double start_y,
                                 double goal_x, double goal_y);

  private:
    // Map-aware coordinate helpers
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
                     double x, double y, int &gx, int &gy) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
                     int gx, int gy, double &x, double &y) const;
    bool isPassable(int8_t cell_value) const;
    // step cost to enter a neighbour
    double stepCost(int8_t neighbour_cell_value, bool diagonal) const;

    rclcpp::Logger logger_;
    int occupancy_threshold_{50};
    double unknown_penalty_{0.5};
  };

}

#endif

#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

    PlannerCore::PlannerCore(const rclcpp::Logger &logger)
        : logger_(logger) {}

    void PlannerCore::configure(int occupancy_threshold, double unknown_penalty)
    {
        occupancy_threshold_ = occupancy_threshold;
        unknown_penalty_ = unknown_penalty;
    }

    bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
                                  double x, double y, int &gx, int &gy) const
    {
        const double res = map.info.resolution;
        const double ox = map.info.origin.position.x;
        const double oy = map.info.origin.position.y;

        gx = static_cast<int>(std::floor((x - ox) / res));
        gy = static_cast<int>(std::floor((y - oy) / res));

        return gx >= 0 && gx < static_cast<int>(map.info.width) && gy >= 0 && gy < static_cast<int>(map.info.height);
    }
    void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
                                  int gx, int gy, double &x, double &y) const
    {
        const double res = map.info.resolution;
        x = map.info.origin.position.x + (gx + 0.5) * res;
        y = map.info.origin.position.y + (gy + 0.5) * res;
    }

    bool PlannerCore::isPassable(int8_t cell_value) const
    {
        return cell_value < occupancy_threshold_;
    }

    double PlannerCore::stepCost(int8_t neighbor_cell_value, bool diagonal) const
    {
        const double base = diagonal ? std::sqrt(2.0) : 1.0;

        if (neighbor_cell_value < 0)
        {
            // unknown -> small penalty so we prefer known free space if possible
            return base + unknown_penalty_;
        }
        return base + (neighbor_cell_value / 10.0);
    }

    nav_msgs::msg::Path PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid &map,
                                              double start_x, double start_y,
                                              double goal_x, double goal_y)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = map.header.frame_id;

        if (map.data.empty())
        {
            RCLCPP_WARN(logger_, "planPath called with an empty map");
            return path;
        }

        // convert start/goal to grid coords
        int sx, sy, gx, gy;
        if (!worldToGrid(map, start_x, start_y, sx, sy))
        {
            RCLCPP_WARN(logger_, "Start outside of map bounds");
            return path;
        }

        if (!worldToGrid(map, goal_x, goal_y, gx, gy))
        {
            RCLCPP_WARN(logger_, "Goal outside map bounds");
            return path;
        }

        CellIndex start(sx, sy);
        CellIndex goal(gx, gy);

        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);

        if (!isPassable(map.data[sy * width + sx]))
        {
            RCLCPP_WARN(logger_, "Start cell is not passable");
            return path;
        }
        if (!isPassable(map.data[gy * width + gx]))
        {
            RCLCPP_WARN(logger_, "Goal cell is not passable");
            return path;
        }

        const int dxs[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dys[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
        std::unordered_map<CellIndex, double, CellIndexHash> g_score;
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

        auto euclidean = [](const CellIndex &a, const CellIndex &b)
        {
            return std::hypot(a.x - b.x, a.y - b.y);
        };

        g_score[start] = 0.0;
        open.emplace(start, euclidean(start, goal));

        bool found = false;

        while (!open.empty())
        {
            AStarNode current = open.top();
            open.pop();

            if (current.index == goal)
            {
                found = true;
                break;
            }

            const double current_g = g_score[current.index];

            for (int i = 0; i < 8; ++i)
            {
                const int nx = current.index.x + dxs[i];
                const int ny = current.index.y + dys[i];
                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    continue;

                const int8_t cell = map.data[ny * width + nx];
                if (!isPassable(cell))
                    continue;

                const bool diagonal = (dxs[i] != 0 && dys[i] != 0);
                if (diagonal)
                {
                    // Prevent corner-cutting through walls: both cardinal neighbours must be passable.
                    const int8_t cardinal_x = map.data[current.index.y * width + (current.index.x + dxs[i])];
                    const int8_t cardinal_y = map.data[(current.index.y + dys[i]) * width + current.index.x];
                    if (!isPassable(cardinal_x) || !isPassable(cardinal_y))
                        continue;
                }
                const double tentative_g = current_g + stepCost(cell, diagonal);

                CellIndex neighbor(nx, ny);
                auto it = g_score.find(neighbor);
                if (it == g_score.end() || tentative_g < it->second)
                {
                    g_score[neighbor] = tentative_g;
                    came_from[neighbor] = current.index;
                    const double f = tentative_g + euclidean(neighbor, goal);
                    open.emplace(neighbor, f);
                }
            }
        }

        if (!found)
        {
            RCLCPP_WARN(logger_, "No path found from (%.2f, %.2f) to (%.2f, %.2f)",
                        start_x, start_y, goal_x, goal_y);
            return path;
        }

        // Reconstruct path: walk came_from from goal back to start
        std::vector<CellIndex> cells;
        CellIndex cur = goal;
        cells.push_back(cur);
        while (cur != start)
        {
            auto it = came_from.find(cur);
            if (it == came_from.end())
                break;
            cur = it->second;
            cells.push_back(cur);
        }
        std::reverse(cells.begin(), cells.end());

        // Convert each cell to a world-frame PoseStamped
        path.poses.reserve(cells.size());
        for (const auto &c : cells)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = map.header.frame_id;
            p.header.stamp = map.header.stamp;
            double wx, wy;
            gridToWorld(map, c.x, c.y, wx, wy);
            p.pose.position.x = wx;
            p.pose.position.y = wy;
            p.pose.orientation.w = 1.0;
            path.poses.push_back(p);
        }

        return path;
    }

}

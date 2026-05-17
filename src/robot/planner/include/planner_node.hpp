#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  enum class State
  {
    WAITING_FOR_GOAL,
    WAITING_FOR_ROBOT_TO_REACH_GOAL
  };

  // CALLBACK
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Helpers
  void planAndPublish();
  bool goalReached() const;

  robot::PlannerCore planner_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_{State::WAITING_FOR_GOAL};
  nav_msgs::msg::OccupancyGrid current_map_;
  bool have_map_{false};
  geometry_msgs::msg::PointStamped goal_;
  bool have_goal_{false};
  double robot_x_{0.0}, robot_y_{0.0};
  bool have_odom_{false};
  rclcpp::Time goal_start_time_;
  rclcpp::Time last_plan_time_;
  bool have_last_plan_time_{false};

  double goal_tolerance_{0.5};
  double timeout_sec_{30.0};
  double replan_min_interval_sec_{1.0};
};

#endif

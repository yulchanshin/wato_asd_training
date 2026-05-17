#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  timeout_sec_ = this->declare_parameter<double>("timeout_sec", 30.0);
  replan_min_interval_sec_ = this->declare_parameter<double>("replan_min_interval_sec", 1.0);
  const int occupancy_threshold = this->declare_parameter<int>("occupancy_threshold", 50);
  const double unknown_penalty = this->declare_parameter<double>("unknown_penalty", 0.5);
  const int timer_period_ms = this->declare_parameter<int>("timer_period_ms", 500);

  planner_.configure(occupancy_threshold, unknown_penalty);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  have_map_ = true;
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
    return;

  if (have_last_plan_time_)
  {
    const double since_last = (this->now() - last_plan_time_).seconds();
    if (since_last < replan_min_interval_sec_)
      return;
  }
  planAndPublish();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  have_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
              goal_.point.x, goal_.point.y);
  planAndPublish();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    return;
  }

  if (goalReached())
  {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  const double elapsed = (this->now() - goal_start_time_).seconds();
  if (elapsed > timeout_sec_)
  {
    RCLCPP_WARN(this->get_logger(), "Goal timeout (%.1fs). Replanning.", elapsed);
    goal_start_time_ = this->now(); // reset clock
    planAndPublish();
  }
}

bool PlannerNode::goalReached() const
{
  if (!have_goal_ || !have_odom_)
    return false;
  const double dx = goal_.point.x - robot_x_;
  const double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planAndPublish()
{
  if (!have_map_ || !have_goal_ || !have_odom_)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan yet (map=%d goal=%d odom=%d)",
                have_map_, have_goal_, have_odom_);
    return;
  }

  auto path = planner_.planPath(current_map_,
                                robot_x_, robot_y_,
                                goal_.point.x, goal_.point.y);
  path.header.stamp = this->now();
  path_pub_->publish(path);
  last_plan_time_ = this->now();
  have_last_plan_time_ = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

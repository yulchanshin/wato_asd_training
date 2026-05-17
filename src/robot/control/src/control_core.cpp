#include "control_core.hpp"

namespace robot
{

  ControlCore::ControlCore(const rclcpp::Logger &logger)
      : logger_(logger)
  {
  }

  double ControlCore::distance(double x1, double y1, double x2, double y2)
  {
    return std::hypot(x2 - x1, y2 - y1);
  }

  void ControlCore::setPath(const nav_msgs::msg::Path &path)
  {
    current_path_ = path;
    have_path_ = !path.poses.empty(); // if empty path is given, have_path_ is still false
  }

  bool ControlCore::hasPath() const
  {
    return have_path_;
  }

  void ControlCore::configure(double lookahead_distance,
                              double goal_tolerance,
                              double linear_speed,
                              double max_angular_speed)
  {
    lookahead_distance_ = lookahead_distance;
    goal_tolerance_ = goal_tolerance;
    linear_speed_ = linear_speed;
    max_angular_speed_ = max_angular_speed;
  }

  bool ControlCore::goal_Reached(double robot_x, double robot_y) const
  {
    if (!have_path_)
    {
      return false;
    }

    const auto &goal_point = current_path_.poses.back().pose.position;
    return distance(robot_x, robot_y, goal_point.x, goal_point.y) < goal_tolerance_;
  }

  bool ControlCore::findLookaheadPoint(double robot_x,
                                       double robot_y,
                                       geometry_msgs::msg::PoseStamped &out) const
  {
    if (!have_path_)
    {
      return false;
    }

    for (const auto &pose : current_path_.poses)
    {
      const auto &point = pose.pose.position;
      if (distance(robot_x, robot_y, point.x, point.y) >= lookahead_distance_)
      {
        out = pose;
        return true;
      }
    }

    out = current_path_.poses.back();
    return true;
  }

  double ControlCore::computeAngularVelocity(double robot_x,
                                             double robot_y,
                                             double robot_yaw,
                                             double lx,
                                             double ly) const
  {
    double dx = lx - robot_x;
    double dy = ly - robot_y;
    double L2 = dx * dx + dy * dy;

    if (L2 < 1e-6)
    {
      return 0.0;
    }

    const double local_y = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;
    const double curvature = 2.0 * local_y / L2;
    double omega = linear_speed_ * curvature;

    if (omega > max_angular_speed_)
    {
      omega = max_angular_speed_;
    }
    if (omega < -max_angular_speed_)
    {
      omega = -max_angular_speed_;
    }
    return omega;
  }

  bool ControlCore::computeCommand(double robot_x,
                                   double robot_y,
                                   double robot_yaw,
                                   geometry_msgs::msg::Twist &cmd_out)
  {
    if (!have_path_)
    {
      return false;
    }

    geometry_msgs::msg::PoseStamped lookahead;
    if (!findLookaheadPoint(robot_x, robot_y, lookahead))
    {
      return false;
    }

    const double lx = lookahead.pose.position.x;
    const double ly = lookahead.pose.position.y;

    cmd_out.linear.x = linear_speed_;
    cmd_out.angular.z = computeAngularVelocity(robot_x, robot_y, robot_yaw, lx, ly);
    return true;
  }

}

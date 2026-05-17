#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:
  // CALLBACKS
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Helpers
  static double quaternionToYaw(double x, double y, double z, double w);
  void publishStop();

  robot::ControlCore control_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Robot pose, populated by odom callback
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  bool have_odom_{false};

  // Params
  double lookahead_distance_{1.0};
  double linear_speed_{0.2};
  double goal_tolerance_{0.3};
  double max_angular_speed_{1.1};
  double control_period_sec_{0.1};
};

#endif

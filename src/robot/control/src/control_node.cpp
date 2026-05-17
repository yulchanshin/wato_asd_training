#include "control_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

ControlNode::ControlNode() : Node("control"), control_(this->get_logger())
{
  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
  linear_speed_ = this->declare_parameter<double>("linear_speed", 0.4);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.3);
  max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.5);
  control_period_sec_ = this->declare_parameter<double>("control_period_sec", 0.1);

  control_.configure(lookahead_distance_, goal_tolerance_, linear_speed_, max_angular_speed_);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(control_period_sec_ * 1000.0));
  timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.setPath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  const auto &q = msg->pose.pose.orientation;
  robot_yaw_ = quaternionToYaw(q.x, q.y, q.z, q.w);

  have_odom_ = true;
}

void ControlNode::timerCallback()
{
  if (!have_odom_ || !control_.hasPath())
  {
    return;
  }

  if (control_.goal_Reached(robot_x_, robot_y_))
  {
    publishStop();
    return;
  }

  geometry_msgs::msg::Twist cmd;
  if (control_.computeCommand(robot_x_, robot_y_, robot_yaw_, cmd))
  {
    cmd_pub_->publish(cmd);
  }
  else
  {
    publishStop();
  }
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

void ControlNode::publishStop()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

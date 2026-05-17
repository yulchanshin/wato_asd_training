#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

  class ControlCore
  {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger &logger);
    void configure(double lookahead_distance,
                   double goal_tolerance,
                   double linear_speed,
                   double max_angular_speed);

    void setPath(const nav_msgs::msg::Path &path);
    bool hasPath() const;
    bool goal_Reached(double robot_x, double robot_y) const;
    bool computeCommand(double robot_x,
                        double robot_y,
                        double robot_yaw,
                        geometry_msgs::msg::Twist &cmd_out);

  private:
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double max_angular_speed_;

    nav_msgs::msg::Path current_path_;
    bool have_path_{false};

    bool findLookaheadPoint(double robot_x, double robot_y,
                            geometry_msgs::msg::PoseStamped &out) const;
    double computeAngularVelocity(double robot_x, double robot_y, double robot_yaw,
                                  double lx, double ly) const;
    static double distance(double x1, double y1, double x2, double y2);
  };

}

#endif

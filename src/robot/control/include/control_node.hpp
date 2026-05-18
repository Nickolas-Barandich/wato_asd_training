#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Stored data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.1;
    double linear_speed_ = 0.5;

    // Callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Pure pursuit helpers
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
    void stopRobot();
};

#endif

#include <chrono>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
 
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10,
              std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,
              std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                   std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    stopRobot();
    return;
  }

  // Find the lookahead point
  std::optional<geometry_msgs::msg::PoseStamped> lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
      stopRobot();
      return;  // No valid lookahead point found
  }

  // Compute velocity command
  geometry_msgs::msg::Twist cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // Safety check
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point robot_position = robot_odom_->pose.pose.position; // Gets robot current positon
  geometry_msgs::msg::PoseStamped final_pose = current_path_->poses.back(); // Gets final waypoint

  // Checks if the robot is close enough to the goal
  if (computeDistance(robot_position, final_pose.pose.position) < goal_tolerance_) {
    stopRobot();
    return std::nullopt;
  }

  // Start from the waypoint closest to the robot. Searching from pose zero
  // eventually selects an old waypoint behind the robot once it has travelled
  // farther than the lookahead distance, which makes it turn back or stall.
  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    const double distance = computeDistance(
        robot_position, current_path_->poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  // Search only forward from the closest point on the path.
  for (size_t i = closest_index; i < current_path_->poses.size(); i++) {
    // Get current waypoint
    const geometry_msgs::msg::PoseStamped& pose = current_path_->poses[i];

    double distance = computeDistance(robot_position, pose.pose.position);

    if (distance >= lookahead_distance_) {
      return pose;
    }
  }
  // If no point is far enough away, aim for the final point
  return final_pose;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  // Get location and orientation of robot
  geometry_msgs::msg::Point robot_position = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // Get difference from robot to target
  double dx = target.pose.position.x - robot_position.x;
  double dy = target.pose.position.y - robot_position.y;

  // Convert target into robot relative coordinates
  double target_x_robot = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  double target_y_robot = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  // Compute look ahead distance via distance formula
  double lookahead_distance = std::sqrt(target_x_robot * target_x_robot + 
                                        target_y_robot * target_y_robot);

  // Prevents weird stuff from happening when dividing at zero or near
  if (lookahead_distance < 0.001) {
    return cmd_vel;
  }

  const double heading_error = std::atan2(target_y_robot, target_x_robot);

  // Pure pursuit becomes unstable for a target far to the side. Turn until
  // the path is in front, then resume forward tracking.
  if (std::abs(heading_error) > turn_in_place_angle_) {
    cmd_vel.angular.z = std::copysign(angular_speed_, heading_error);
    return cmd_vel;
  }

  if (target_x_robot < 0.0) {
    cmd_vel.linear.x = 0.0;

    if (target_y_robot >= 0.0) {
      cmd_vel.angular.z = angular_speed_;
    } else {
      cmd_vel.angular.z = -angular_speed_;
    }

    return cmd_vel;
  }

  if (std::abs(target_y_robot) < 0.001) {
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }
  // Run faster on straight segments for recording, then taper forward speed
  // as the heading error grows. Angular velocity uses the same commanded
  // speed, preserving the requested Pure Pursuit curvature.
  const double speed_scale = std::clamp(
      1.0 - 0.5 * std::abs(heading_error) / turn_in_place_angle_,
      0.5, 1.0);
  const double commanded_linear_speed = linear_speed_ * speed_scale;

  // Find circle radius and then use curvature = 1/radius
  double circle_radius = (lookahead_distance * lookahead_distance) / (2.0 * target_y_robot);
  double curvature = 1.0 / circle_radius;

  // Using curvature, computes velocity
  cmd_vel.linear.x = commanded_linear_speed;
  cmd_vel.angular.z = std::clamp(commanded_linear_speed * curvature,
                                 -angular_speed_, angular_speed_);

    return cmd_vel;
}

// Compute distance using distance formula
double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;

  return std::sqrt(dx * dx + dy * dy);
}

// Quaternion to yaw formula
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

// Stops the robot
void ControlNode::stopRobot(){
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_odom_ = msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

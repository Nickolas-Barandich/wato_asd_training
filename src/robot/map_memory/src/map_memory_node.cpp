#include <cmath>
#include <functional>
#include <memory>
#include <chrono>

#include "map_memory_node.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Initialize costmap subscriber
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10,
                 std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // Initialize odometry subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,
              std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;

    // Debug statement
    //RCLCPP_INFO(this->get_logger(), "Received costmap with %zu cells", msg->data.size());
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  odom_received_ = true;

  geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;

  // Quaternion to yaw formula
  current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Compute distance traveled
  double distance = std::sqrt(std::pow(current_x_ - last_x_, 2) + std::pow(current_y_ - last_y_, 2));

  if (distance >= distance_threshold_) {
      last_x_ = current_x_;
      last_y_ = current_y_;
      should_update_map_ = true;
  }
  // Debug Statement
  //RCLCPP_INFO(this->get_logger(), "Robot moved %.2f meters. Ready to update map.", distance);
}

// Timer-based map update
void MapMemoryNode::updateMap() {
    if (odom_received_ && costmap_updated_ && (should_update_map_ || global_map_.data.empty())) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;

      // Debug Statement
      //RCLCPP_INFO(this->get_logger(), "Published /map with %zu cells", global_map_.data.size());

      // Debug Code tracking number of nonzero cells (testing if memory works)
      /*
      int nonzero_count = 0;
      for (size_t i = 0; i < global_map_.data.size(); i++) {
        if (global_map_.data[i] > 0) {
          nonzero_count++;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Map nonzero cells: %d", nonzero_count);
      */
    }
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  // If there is no data in global map use latest cost map
if (global_map_.data.empty()) {
  global_map_.header.frame_id = "sim_world";
  global_map_.header.stamp = this->get_clock()->now();

  global_map_.info = latest_costmap_.info;

  double map_width_meters =
      global_map_.info.width * global_map_.info.resolution;

  double map_height_meters =
      global_map_.info.height * global_map_.info.resolution;

  // Center the global map around the robot's current odom position
  global_map_.info.origin.position.x =
      current_x_ - map_width_meters / 2.0;

  global_map_.info.origin.position.y =
      current_y_ - map_height_meters / 2.0;

  global_map_.info.origin.position.z = 0.0;

  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(
      static_cast<size_t>(global_map_.info.width) *
      static_cast<size_t>(global_map_.info.height),
      0);

  RCLCPP_INFO(
      this->get_logger(),
      "Initialized global map origin: (%.2f, %.2f), robot: (%.2f, %.2f)",
      global_map_.info.origin.position.x,
      global_map_.info.origin.position.y,
      current_x_,
      current_y_);
}

  // Run through the latest cost map
  for (int y = 0; y < latest_costmap_.info.height; y++) {
    for (int x = 0; x < latest_costmap_.info.width; x++) {

      // Convert the cells into meters
      double local_x = latest_costmap_.info.origin.position.x + (x + 0.5) * latest_costmap_.info.resolution;
      double local_y = latest_costmap_.info.origin.position.y + (y + 0.5) * latest_costmap_.info.resolution;
      
      // Accounting for yaw
      double global_x = current_x_ + std::cos(current_yaw_) * local_x - std::sin(current_yaw_) * local_y;
      double global_y = current_y_ + std::sin(current_yaw_) * local_x + std::cos(current_yaw_) * local_y;

      // Convert back to grid coordinates
      int global_grid_x = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_grid_y = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);
      
      // skip process if global grid coordinates are out of bounds
      if (global_grid_x < 0 || global_grid_x >= static_cast<int>(global_map_.info.width) ||
          global_grid_y < 0 || global_grid_y >= static_cast<int>(global_map_.info.height)) {
        continue;
      }

      // Convert both local and global into 1D arrays
      int global_index = global_grid_x + global_grid_y * global_map_.info.width;
      int local_index = x + y * latest_costmap_.info.width;

      int8_t local_value = latest_costmap_.data[local_index];

      global_map_.data[global_index] = local_value;
    }
  }

    global_map_.header.stamp = this->get_clock()->now();
    global_map_.header.frame_id = "sim_world";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
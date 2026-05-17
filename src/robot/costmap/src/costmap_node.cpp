#include <memory>
#include <cmath>
#include <vector>
#include <utility>
#include <functional>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Subscribe to lidar scans
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10,
               std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// Initializes a 2D array [y][x] and sets all values to zero
void CostmapNode::initializeCostmap(){
  costmap_grid_.assign(height_, std::vector<int8_t>(width_, 0));
}

// Converts polar coordinates to grid
void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid){
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>(origin_x_grid_ + (x / resolution_));
  y_grid = static_cast<int>(origin_y_grid_ + (y / resolution_));
}

// Marks the obstacle at the max cost
void CostmapNode::markObstacle(int x_grid, int y_grid){
  if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_) {
    return; // not in bounds so return
  }
  costmap_grid_[y_grid][x_grid] = max_cost_;
}

void CostmapNode::inflateObstacles(){
  // Calculate the radius of cells affected by the inflation radius
  int inflation_radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  // Stores x and y coordinates of all obstacles
  std::vector<std::pair<int, int>> obstacle_cells;

  // Find the original obstacle cells
  for (int y = 0; y < height_; y++){
    for (int x = 0; x < width_; x++){
      if (costmap_grid_[y][x] == max_cost_){
        obstacle_cells.push_back({x, y});
      }
    }
  }

  // Then inflate around each original obstacle
  for (size_t i = 0; i < obstacle_cells.size(); i++) {

    int obstacle_x = obstacle_cells[i].first;
    int obstacle_y = obstacle_cells[i].second;

    for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; dy++){
      for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; dx++){
        int new_x = obstacle_x + dx;
        int new_y = obstacle_y + dy;

        // Skip cells outside the map
        if (new_x < 0 || new_x >= width_ || new_y < 0 || new_y >= height_) {
          continue;
        }

        // Distance from obstacle cell to this nearby cell
        double distance_cells = std::sqrt((dx * dx) + (dy * dy));
        double distance_meters = distance_cells * resolution_;
        
        // Skip cells outside the inflation radius
        if (distance_meters > inflation_radius_) {
          continue;
        }

        int cost = static_cast<int>(max_cost_ * (1.0 - (distance_meters / inflation_radius_)));

        // Only increase a cost, never lower it
        if (cost > costmap_grid_[new_y][new_x]) {
          costmap_grid_[new_y][new_x] = cost;
        }
      }
    }
  }
}

// publishes the costmap to /costmap
void CostmapNode::publishCostmap(){
  nav_msgs::msg::OccupancyGrid costmap_msg;

  // Header
  costmap_msg.header.stamp = this->get_clock()->now();
  costmap_msg.header.frame_id = "robot/chassis/lidar";

  /* 
  The origin is the real-world position of grid cell [0][0].
  Since the robot/lidar is placed in the at the center of the grid,
  the bottom-left corner is negative half the map size.
  */
  costmap_msg.info.origin.position.x = -origin_x_grid_ * resolution_;
  costmap_msg.info.origin.position.y = -origin_y_grid_ * resolution_;
  costmap_msg.info.origin.position.z = 0.0;

  costmap_msg.info.resolution = resolution_;
  costmap_msg.info.width = width_;
  costmap_msg.info.height = height_;

  costmap_msg.info.origin.orientation.x = 0.0;
  costmap_msg.info.origin.orientation.y = 0.0;
  costmap_msg.info.origin.orientation.z = 0.0;
  costmap_msg.info.origin.orientation.w = 1.0;

  // ensures that data array starts empty
  costmap_msg.data.clear();

  // Flattens the 2D array into a 1D array
  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
    costmap_msg.data.push_back(costmap_grid_[y][x]);
    }
  }

  costmap_pub_->publish(costmap_msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {

        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range < scan->range_max && range > scan->range_min) {
            
          // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
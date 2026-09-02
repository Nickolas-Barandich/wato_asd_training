#include <memory>
#include <cmath>
#include <vector>
#include <utility>
#include <functional>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  inflation_radius_ = this->declare_parameter<double>("inflation_radius", 2.25);

  // Subscribe to lidar scans
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10,
               std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// Unknown cells must remain distinct from cells that a lidar ray observed as
// free. This lets map memory clear stale readings without erasing unseen walls.
void CostmapNode::initializeCostmap(){
  costmap_grid_.assign(height_, std::vector<int8_t>(width_, -1));
}

// Bresenham ray trace from the lidar to (but not including) the endpoint.
void CostmapNode::markRayFree(int end_x, int end_y) {
  int x = origin_x_grid_;
  int y = origin_y_grid_;
  const int dx = std::abs(end_x - x);
  const int sx = x < end_x ? 1 : -1;
  const int dy = -std::abs(end_y - y);
  const int sy = y < end_y ? 1 : -1;
  int error = dx + dy;

  while (x != end_x || y != end_y) {
    // Laser samples fan apart with range (about five cells at 20 m in this
    // simulation). Rasterize a narrow brush around each ray so observed free
    // space is a continuous region instead of radial one-cell stripes.
    for (int offset_y = -free_ray_padding_cells_;
         offset_y <= free_ray_padding_cells_; ++offset_y) {
      for (int offset_x = -free_ray_padding_cells_;
           offset_x <= free_ray_padding_cells_; ++offset_x) {
        const int free_x = x + offset_x;
        const int free_y = y + offset_y;
        if (free_x >= 0 && free_x < width_ &&
            free_y >= 0 && free_y < height_) {
          costmap_grid_[free_y][free_x] = 0;
        }
      }
    }
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y += sy;
    }
  }
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
 
    std::vector<std::pair<int, int>> obstacle_cells;

    // Step 2: Rasterize observed free space for every scan ray.
    for (size_t i = 0; i < scan->ranges.size(); ++i) {

        double angle = scan->angle_min + i * scan->angle_increment;
        const double measured_range = scan->ranges[i];
        if (std::isnan(measured_range) || measured_range <= scan->range_min) {
          continue;
        }

        const bool obstacle_hit = std::isfinite(measured_range) &&
                                  measured_range < scan->range_max;
        const double ray_range = obstacle_hit ? measured_range : scan->range_max;

        int x_grid, y_grid;
        convertToGrid(ray_range, angle, x_grid, y_grid);
        markRayFree(x_grid, y_grid);
        if (obstacle_hit) {
          obstacle_cells.push_back({x_grid, y_grid});
        }
    }

    // Step 3: Restore obstacle endpoints after all free-space rasterization.
    // A neighbouring thick ray must never erase a real lidar return.
    for (const auto& obstacle : obstacle_cells) {
      markObstacle(obstacle.first, obstacle.second);
    }
 
    // Step 4: Inflate obstacles
    inflateObstacles();
 
    // Step 5: Publish costmap
    publishCostmap();
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

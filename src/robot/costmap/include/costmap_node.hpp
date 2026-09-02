#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
   
  private:
    // Callback
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Helper Functions
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markRayFree(int end_x, int end_y);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();

    // The costmap data
    std::vector<std::vector<int8_t>> costmap_grid_;

    // Parameters for Costmap
    double resolution_ = 0.1; // meters per cell
    int width_ = 400; // number of grid cells
    int height_ = 400; // number of grid cells
    int origin_x_grid_ = width_ / 2;
    int origin_y_grid_ = height_ / 2;
    double inflation_radius_= 1.0; // inflation radius (in meters)
    int free_ray_padding_cells_ = 2;
    int max_cost_ = 100;

    robot::CostmapCore costmap_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

};
 
#endif

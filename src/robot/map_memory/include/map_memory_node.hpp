#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    // Callback/helper functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    void integrateCostmap();

    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stored map data
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::OccupancyGrid global_map_;

    // Frame for the remembered/global map
    std::string map_frame_id_ = "sim_world";

    // Robot position tracking
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double distance_threshold_ = 1.5;

    // Flags
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    bool odom_received_ = false;
};

#endif 
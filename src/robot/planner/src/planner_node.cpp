#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
             "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
              "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
              "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
           std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  map_received_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  goal_ = *msg;
  goal_received_ = true;

  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
  
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
          goal_received_ = false;
      } else {
          RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
          planPath();
      }
  }
}

bool PlannerNode::goalReached(){
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_threshold_;
}

void PlannerNode::planPath(){
  if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
      return;
  }

  // Convert start and goal positions to grid cells
  CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y);

  if (isCellInBounds(start)) {
  int start_index = start.x + start.y * current_map_.info.width;
  int8_t start_value = current_map_.data[start_index];

  RCLCPP_INFO(
      this->get_logger(),
      "Start cell: (%d, %d), value: %d, robot: (%.2f, %.2f)",
      start.x,
      start.y,
      static_cast<int>(start_value),
      robot_pose_.position.x,
      robot_pose_.position.y);
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Start cell out of bounds: (%d, %d), robot: (%.2f, %.2f)",
          start.x,
          start.y,
          robot_pose_.position.x,
          robot_pose_.position.y);
    }

  // Confirm the start cell is free
  if (!isCellFree(start)) {
    RCLCPP_WARN(this->get_logger(), "Start cell is not free!");
    return;
  }

    if (!isCellInBounds(goal)) {
    RCLCPP_WARN(
        this->get_logger(),
        "Goal cell out of bounds: (%d, %d), goal: (%.2f, %.2f)",
        goal.x,
        goal.y,
        goal_.point.x,
        goal_.point.y);
    return;
  }

  // Confirm the goal cell is free
  if (!isCellFree(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal cell is not free!");
    return;
  }

  // Run A*
  std::vector<CellIndex> grid_path = runAStar(start, goal);

  // If A* fails
  if (grid_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found.");
    return;
  }

  // Create ROS path message
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id;

  // Convert grid cells to Poses
  for (const CellIndex& cell : grid_path) {
    path.poses.push_back(gridToPose(cell));
  }
  
  // Publish path
  path_pub_->publish(path);

  // debug info
  RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path.poses.size());
}

// Converts to grid coordinates
CellIndex PlannerNode::worldToGrid(double world_x, double world_y){
  int grid_x = static_cast<int>((world_x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int grid_y = static_cast<int>((world_y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  
  return CellIndex(grid_x, grid_y);
}

// converts grid back to pose
geometry_msgs::msg::PoseStamped PlannerNode::gridToPose(const CellIndex& cell){
  geometry_msgs::msg::PoseStamped pose;

  pose.header.stamp = this->get_clock()->now();
  pose.header.frame_id = current_map_.header.frame_id;

  pose.pose.position.x = current_map_.info.origin.position.x +
                         (cell.x + 0.5) * current_map_.info.resolution;
  
  pose.pose.position.y = current_map_.info.origin.position.y +
                         (cell.y + 0.5) * current_map_.info.resolution;
  
  
  pose.pose.position.z = 0.0;

  pose.pose.orientation.w = 1.0;

  return pose;
}

// Checks if cell is in bounds or not
bool PlannerNode::isCellInBounds(const CellIndex& cell){
  return cell.x >= 0 &&
         cell.x < static_cast<int>(current_map_.info.width) &&
         cell.y >= 0 &&
         cell.y < static_cast<int>(current_map_.info.height);
}

bool PlannerNode::isCellFree(const CellIndex& cell){
  if (!isCellInBounds(cell)) {
    return false;
  }

  int index = cell.x + cell.y * current_map_.info.width;
  int8_t value = current_map_.data[index];

  // If value is less than threshold then it is free
  return value < cost_threshold_;
}

// Calculate the distance to the end node
double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b){
  
  // Use euclidean distance
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerNode::getNeighbours(const CellIndex& cell){
  std::vector<CellIndex> neighbours;

  std::vector<CellIndex> directions = {
    CellIndex(1, 0),
    CellIndex(-1, 0),
    CellIndex(0, 1),
    CellIndex(0, -1),
  };

  for (size_t i = 0; i < directions.size(); i++) {
    const CellIndex& dir = directions[i];

    CellIndex neighbour(cell.x + dir.x, cell.y + dir.y);

    if (isCellFree(neighbour)) {
      neighbours.push_back(neighbour);
    }
  }

  return neighbours;
}

// Runs A* search algorithm
std::vector<CellIndex> PlannerNode::runAStar(const CellIndex& start, const CellIndex& goal){
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  // Initialize the start
  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  // Runs until it goes through the entire open set
  while (!open_set.empty()){
    CellIndex current = open_set.top().index;
    open_set.pop();
    
    // If the goal has been reached
    if (current == goal) {
      // Start the path with the goal cell
      std::vector<CellIndex> path;
      path.push_back(current);

      // Walk backwards until you reach the path
      while (came_from.find(current) != came_from.end()) {
        current = came_from[current];
        path.push_back(current);
      }

      // Reverse it so it is start -> goal
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    // Skip already processed cells
    if (closed_set.find(current) != closed_set.end()) {
      continue;
    }

    // Mark current as processed
    closed_set.insert(current);

    std::vector<CellIndex> neighbours = getNeighbours(current);
    // Checks all neighbours
    for (size_t i = 0; i < neighbours.size(); i++) {
      const CellIndex& neighbour = neighbours[i];

      // skip already processed neighbours
      if (closed_set.find(neighbour) != closed_set.end()) {
        continue;
      }

      // calculates new posssible cost
      double tentative_g_score = g_score[current] + 1.0;

      // Check if this path is better
      if (g_score.find(neighbour) == g_score.end() || tentative_g_score < g_score[neighbour]) {
        // Store and update the route
        came_from[neighbour] = current;
        g_score[neighbour] = tentative_g_score;

        // Calculates f score and push it to the open set
        double f_score = tentative_g_score + heuristic(neighbour, goal);
        open_set.push(AStarNode(neighbour, f_score));
      }
    }
  }
  // If no path is found
  RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
  return {};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

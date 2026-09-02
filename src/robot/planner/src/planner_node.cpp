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
  if (!msg->header.frame_id.empty() && map_received_ &&
      msg->header.frame_id != current_map_.header.frame_id) {
    RCLCPP_WARN(this->get_logger(),
      "Rejected goal in frame '%s'; expected '%s' (frame transforms are not available)",
      msg->header.frame_id.c_str(), current_map_.header.frame_id.c_str());
    goal_received_ = false;
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  goal_ = *msg;
  goal_received_ = true;

  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
  
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  const bool first_odometry = !odom_received_;
  robot_pose_ = msg->pose.pose;
  odom_received_ = true;

  if (first_odometry && goal_received_ && map_received_) {
    planPath();
  }
}

void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
          goal_received_ = false;
      }
  }
}

bool PlannerNode::goalReached(){
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_threshold_;
}

void PlannerNode::planPath(){
  if (!goal_received_ || !map_received_ || !odom_received_ || current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map, odometry, or goal");
      publishEmptyPath();
      return;
  }

  if (!goal_.header.frame_id.empty() &&
      goal_.header.frame_id != current_map_.header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan goal in frame '%s' on map frame '%s'",
                goal_.header.frame_id.c_str(), current_map_.header.frame_id.c_str());
    publishEmptyPath();
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
    publishEmptyPath();
    return;
  }

  const int start_index = start.x + start.y * current_map_.info.width;
  const int start_cost = current_map_.data[start_index];

  // The robot may already be inside the conservative inflation band after a
  // turn or a map update. It must be allowed to move down the cost gradient
  // back into free space; rejecting the start creates a permanent deadlock.
  if (start_cost >= lethal_cost_) {
    RCLCPP_WARN(this->get_logger(),
                "Start cell is lethal (cost %d); refusing to plan through an obstacle",
                start_cost);
    publishEmptyPath();
    return;
  }

  if (start_cost >= start_recovery_max_cost_) {
    RCLCPP_WARN(this->get_logger(),
                "Start cell cost %d is too deep in inflation for automatic recovery",
                start_cost);
    publishEmptyPath();
    return;
  }

  CellIndex planning_start = start;
  std::vector<CellIndex> escape_prefix;
  if (start_cost >= cost_threshold_) {
    RCLCPP_WARN(this->get_logger(),
                "Start cell is in the inflated zone (cost %d); planning a decreasing-cost escape",
                start_cost);
    escape_prefix = findStartEscape(start);
    if (escape_prefix.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "No safe decreasing-cost escape found within %.2f m",
                  start_recovery_max_cells_ * current_map_.info.resolution);
      publishEmptyPath();
      return;
    }
    planning_start = escape_prefix.back();
  }

  if (!isCellInBounds(goal)) {
    RCLCPP_WARN(
        this->get_logger(),
        "Goal cell out of bounds: (%d, %d), goal: (%.2f, %.2f)",
        goal.x,
        goal.y,
        goal_.point.x,
        goal_.point.y);
    publishEmptyPath();
    return;
  }

  // Confirm the goal cell is free
  if (!isCellFree(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal cell is not free!");
    publishEmptyPath();
    return;
  }

  // Run A*
  std::vector<CellIndex> grid_path = runAStar(planning_start, goal);

  // If A* fails
  if (grid_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found.");
    publishEmptyPath();
    return;
  }

  if (!escape_prefix.empty()) {
    // The last escape cell is also the first normal A* cell.
    escape_prefix.pop_back();
    grid_path.insert(grid_path.begin(), escape_prefix.begin(), escape_prefix.end());
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

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = map_received_ ? current_map_.header.frame_id : "sim_world";
  path_pub_->publish(path);
}

// Converts to grid coordinates
CellIndex PlannerNode::worldToGrid(double world_x, double world_y){
  int grid_x = static_cast<int>(std::floor(
      (world_x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  int grid_y = static_cast<int>(std::floor(
      (world_y - current_map_.info.origin.position.y) / current_map_.info.resolution));
  
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

std::vector<CellIndex> PlannerNode::findStartEscape(const CellIndex& start) {
  struct EscapeNode {
    CellIndex cell;
    int depth;
  };

  std::queue<EscapeNode> open;
  std::unordered_set<CellIndex, CellIndexHash> visited;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  open.push({start, 0});
  visited.insert(start);

  const std::vector<CellIndex> directions = {
    CellIndex(1, 0), CellIndex(-1, 0),
    CellIndex(0, 1), CellIndex(0, -1),
  };

  while (!open.empty()) {
    const EscapeNode current = open.front();
    open.pop();

    if (current.cell != start && isCellFree(current.cell)) {
      std::vector<CellIndex> path{current.cell};
      CellIndex cursor = current.cell;
      while (cursor != start) {
        cursor = came_from[cursor];
        path.push_back(cursor);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    if (current.depth >= start_recovery_max_cells_) {
      continue;
    }

    const int current_index =
        current.cell.x + current.cell.y * current_map_.info.width;
    const int current_cost = current_map_.data[current_index];

    for (const CellIndex& direction : directions) {
      const CellIndex neighbour(
          current.cell.x + direction.x, current.cell.y + direction.y);
      if (!isCellInBounds(neighbour) || visited.count(neighbour) > 0) {
        continue;
      }

      const int neighbour_index =
          neighbour.x + neighbour.y * current_map_.info.width;
      const int neighbour_cost = current_map_.data[neighbour_index];
      const bool safe_free_cell = isCellFree(neighbour);
      const bool decreasing_inflation =
          neighbour_cost >= cost_threshold_ &&
          neighbour_cost < start_recovery_max_cost_ &&
          neighbour_cost <= current_cost;

      if (!safe_free_cell && !decreasing_inflation) {
        continue;
      }

      visited.insert(neighbour);
      came_from[neighbour] = current.cell;
      open.push({neighbour, current.depth + 1});
    }
  }

  return {};
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

      const int neighbour_index =
          neighbour.x + neighbour.y * current_map_.info.width;
      const int8_t cell_cost = current_map_.data[neighbour_index];

      // Unknown space remains traversable so distant goals are reachable, but
      // it is substantially less desirable than space actually cleared by a
      // lidar ray. This prevents A* from cutting through the unseen interior
      // and shadow of an obstacle when a confirmed-free route exists around it.
      const double traversal_penalty =
          cell_cost < 0 ? 5.0 : static_cast<double>(cell_cost) / cost_threshold_;
      double tentative_g_score = g_score[current] + 1.0 + traversal_penalty;

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

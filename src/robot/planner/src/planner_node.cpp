#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
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

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}
 
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}
 
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}
 
void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}
 
bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}
 
CellIndex PlannerNode::poseToCell(double x, double y) {
    int ix = std::floor((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int iy = std::floor((y - current_map_.info.origin.position.y) / current_map_.info.resolution);
    return CellIndex(ix, iy);
}

double calculate_H(CellIndex A, CellIndex B) {
  return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
 
  // A* Implementation (pseudo-code)
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";
 
  // Compute path using A* on current_map_
  // Fill path.poses with the resulting waypoints.
 
  CellIndex start_idx = poseToCell(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal_idx = poseToCell(goal_.point.x, goal_.point.y);

  //convert to cell index

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openList;
  std::unordered_map<CellIndex, bool, CellIndexHash> closedList;

  std::unordered_map<CellIndex, AStarNode, CellIndexHash> mapping;

  //std::unordered_map<int,int> g_num;

  AStarNode starter(start_idx, 0.0, calculate_H(start_idx, goal_idx));
  // starter.index.x = current_map_.info.origin.position.x;
  // starter.index.y = current_map_.info.origin.position.y;

  //starter.from = -99999;
  // starter.g_score = 0;
  // starter.h_score = sqrt(pow(goal_.point.x - starter.index.x , 2) + pow(goal_.point.y - starter.index.y , 2));
  // starter.f_score = starter.g_score + starter.h_score;

  openList.push(starter);
  mapping[start_idx] = starter;

  int dx[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  int dy[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  double costs[8] = {1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4};

  bool flag = false;
  while(!openList.empty()) {
    AStarNode cur = openList.top();
    openList.pop();

    if (closedList.count(cur.index)) {
      continue;
    }
    closedList[cur.index] = true;

    if(cur.index == goal_idx) {
      flag = true;
      break;
    }

    //int mapping_num = CellIndexHash(cur.index);
    //mapping[mapping_num] = cur;

    for(int i = 0; i < 8; i++) {
      int x = dx[i] + cur.index.x;
      int y = dy[i] + cur.index.y;

      // int index_j = (x - current_map_.info.origin.position.x)/current_map_.info.resolution;
      // int index_i = (y - current_map_.info.origin.position.y)/current_map_.info.resolution;

      CellIndex nxt(x,y);

      if(x < 0 || y < 0 || x >= current_map_.info.width || y >= current_map_.info.height) {
        continue;
      }
      if(current_map_.data[y * current_map_.info.width + x] > 30 ) {
        continue;
      }
      if(closedList.count(nxt)) {
        continue;
      }

      double map_cost_penalty = current_map_.data[y * current_map_.info.width + x] / 100.0;
      double step_cost = costs[i] + (map_cost_penalty * 3.0);

      if(mapping.find(nxt) == mapping.end() || cur.g_score + step_cost < mapping[nxt].g_score) {
        AStarNode nxtA(nxt, cur.g_score + step_cost, calculate_H(nxt, goal_idx));
        nxtA.from = cur.index;
        mapping[nxt] = nxtA;
        openList.push(nxtA);
      }

    }
  }

  if(flag == true) {
    CellIndex cur = goal_idx;
    while(cur != start_idx) {
      geometry_msgs::msg::PoseStamped item;
      item.pose.position.x = (cur.x+0.5) * current_map_.info.resolution + current_map_.info.origin.position.x;
      item.pose.position.y = (cur.y+0.5) * current_map_.info.resolution + current_map_.info.origin.position.y;
      item.header.frame_id = "map";
      item.header.stamp = this->get_clock()->now();
      
      path.poses.push_back(item);
      cur = mapping[cur].from;
    }
    std::reverse(path.poses.begin(), path.poses.end());
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

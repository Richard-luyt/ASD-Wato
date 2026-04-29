#include "control_node.hpp"

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // TODO: Implement logic to find the lookahead point on the path
  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;

  for(auto it = current_path_->poses.begin(); it != current_path_->poses.end(); it++) {
    double cur_x = it->pose.position.x;
    double cur_y = it->pose.position.y;
    double distance = sqrt(pow(cur_x - robot_x , 2) + pow(cur_y - robot_y, 2));
    if (distance >= lookahead_distance_ || abs(distance - lookahead_distance_) <= 1e-6) {
      geometry_msgs::msg::PoseStamped newPose;
      newPose.pose.position.x = cur_x;
      newPose.pose.position.y = cur_y;
      return newPose;
    }
  }

  if (!current_path_->poses.empty()) {
      geometry_msgs::msg::PoseStamped newPose;
      newPose.pose.position.x = current_path_->poses.back().pose.position.x;
      newPose.pose.position.y = current_path_->poses.back().pose.position.y;
      return newPose;
  }

  return std::nullopt;  // Replace with a valid point when implemented
}
 
geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;
  double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;
  double theta = atan2(dy,dx);

  double alpha = theta - extractYaw(robot_odom_->pose.pose.orientation);
  double k = 2 * sin(alpha) / computeDistance(robot_odom_->pose.pose.position, target.pose.position);
  const double L = 1.3;
  double steering = atan(L * k);
  cmd_vel.angular.z = steering;
  cmd_vel.linear.x = linear_speed_;
  return cmd_vel;
}
 
double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  // TODO: Implement distance calculation between two points
  double x = a.x - b.x;
  double y = a.y - b.y;
  //return 0.0;
  return sqrt(pow(x,2)+pow(y,2));
}
 
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  double w = quat.w;
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double angle = atan2(2 * (w * z + x * y), 1 - 2*(y*y + z*z));
  return angle;
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
    return;
  }

  if (current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stopping;
    stopping.linear.x = 0;
    stopping.angular.z = 0;
    cmd_vel_pub_->publish(stopping);
    return;
  }

  geometry_msgs::msg::PoseStamped target = current_path_->poses.back();
  if(computeDistance(robot_odom_->pose.pose.position, target.pose.position) <= goal_tolerance_) {
    geometry_msgs::msg::Twist stopping;
    stopping.linear.x = 0;
    stopping.angular.z = 0;
    cmd_vel_pub_->publish(stopping);
    return;
  }
 
  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    geometry_msgs::msg::Twist stopping;
    stopping.linear.x = 0;
    stopping.angular.z = 0;
    cmd_vel_pub_->publish(stopping);
    return;  // No valid lookahead point found
  }
    // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);
 
  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters
  lookahead_distance_ = 1.0;  // Lookahead distance
  goal_tolerance_ = 0.1;     // Distance to consider the goal reached
  linear_speed_ = 0.5;       // Constant forward speed
 
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
 
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
 
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
  // Timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

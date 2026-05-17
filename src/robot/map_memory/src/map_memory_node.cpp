#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  last_x = 0;
  last_y = 0;
  distance_threshold = 0.2;
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
 
  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
 
  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  global_map_height = 500;
  global_map_width = 500;
  global_res = 0.1;

  global_map_.header.frame_id = "map";

  global_map_.info.width = 500;
  global_map_.info.height = 500;
  global_map_.info.resolution = 0.1;

  global_map_.info.origin.position.x = -(global_map_width * global_res) / 2.0;
  global_map_.info.origin.position.y = -(global_map_height * global_res) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(global_map_width * global_map_height, -1);
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the latest costmap
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}
 
// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
 
  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold) {
    double qw = msg->pose.pose.orientation.w;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    yaw = atan2(2 * (qw * qz + qx * qy), 1-2 * (qy * qy + qz * qz));
    last_x = x;
    last_y = y;
    should_update_map_ = true;
  }
}
 
// Timer-based map update
void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}
 
// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  // Transform and merge the latest costmap into the global map
  // (Implementation would handle grid alignment and merging logic)
  //should_update_map_ = false;
  int i = 0;
  int j = 0;
  for(auto it = latest_costmap_.data.begin(); it != latest_costmap_.data.end(); it++) {
    double dx = j * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.x;
    double dy = i * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.y;
    
    int local_index = it - latest_costmap_.data.begin();

    double x_rotate = dx * cos(yaw) - dy * sin(yaw);
    double y_rotate = dx * sin(yaw) + dy * cos(yaw);

    double x_global = last_x + x_rotate;
    double y_global = last_y + y_rotate;

    double x_map = x_global - global_map_.info.origin.position.x;
    double y_map = y_global - global_map_.info.origin.position.y;

    int new_i = std::floor(y_map/global_res);
    int new_j = std::floor(x_map/global_res);

    if(new_i < 0 || new_j < 0 || new_i >= global_map_height || new_i >= global_map_width) {
      continue;
    }

    int global_index = new_i * global_map_width + new_j;

    int local_val = latest_costmap_.data[local_index];
    int global_val = global_map_.data[global_index];

    if (local_val != -1) {
        for (int di = 0; di <= 1; ++di) {
            for (int dj = 0; dj <= 1; ++dj) {
                int fill_i = new_i + di;
                int fill_j = new_j + dj;

                if (fill_i >= 0 && fill_i < global_map_height && fill_j >= 0 && fill_j < global_map_width) {
                    int global_index = fill_i * global_map_width + fill_j;
                    int global_val = global_map_.data[global_index];
                    if (local_val > global_val || global_val == -1) {
                        global_map_.data[global_index] = local_val;
                    }
                }
            }
        }
    }

    j++;
    if(j == latest_costmap_.info.width) {
      i++;
      j = 0;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

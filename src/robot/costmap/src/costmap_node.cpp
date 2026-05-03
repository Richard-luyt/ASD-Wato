#include <chrono>
#include <memory>
//#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  width = 100;
  height = 100;
  resolution = 0.1;
  inflation_rad = 1.3;
  max_cost = 100;

  OccupancyGrid.resize(100, std::vector<int>(100, 0));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
  ("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

}
 
// // Define the timer to publish a message every 500ms
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }



void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

    X.clear();
    Y.clear();   
    for (auto &it : OccupancyGrid) {
      std::fill(it.begin(), it.end(), 0);
    }
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            double x = range * cos(angle);
            double y = range * sin(angle);

            int x_grid, y_grid;
            
            x_grid = (x / resolution) + (width / 2);
            y_grid = (y / resolution) + (height / 2);

            if(x_grid < 0 || x_grid >= width || y_grid >= height || y_grid < 0) {
              continue;
            }

            OccupancyGrid[y_grid][x_grid] = max_cost;
            X.push_back(x_grid);
            Y.push_back(y_grid);

        }
    }
    auto ity = Y.begin();
    for(auto itx = X.begin(); itx != X.end(); itx++) {
      int x = *itx;
      int y = *ity;
      int up_y = y - inflation_rad / resolution;
      int down_y = y + inflation_rad / resolution;
      int left_x = x - inflation_rad / resolution;
      int right_x = x + inflation_rad / resolution;
      for (int i = up_y; i <= down_y; i++) {
        for (int j = left_x; j <= right_x; j++) {
          if(i < 0 || j < 0 || i >= height || j >= width) {
            continue;
          }
          double distance = sqrt((i - y) * (i - y) + (j - x) * (j - x));
          distance = distance * resolution;
          if (distance < inflation_rad || abs(distance - inflation_rad) < 1e-6) {
            // in radius
            int cost = max_cost * (1.0 - distance / inflation_rad);
            if(OccupancyGrid[i][j] < cost) {
              OccupancyGrid[i][j] = cost;
            }
            //OccupancyGrid[i][j] = max(OccupancyGrid[i][j], cost);
          }
        }
      }
      ity++;
    }
 
    // Step 4: Publish costmap
    //publishCostmap();

    auto map = nav_msgs::msg::OccupancyGrid();
    map.header = scan->header;
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;

    map.info.origin.position.x = -(width * resolution) / 2.0;
    map.info.origin.position.y = -(height * resolution) / 2.0;

    map.data.resize(width * height);
    for(int i = 0; i < height; i++) {
      for(int j = 0; j < width; j++) {
        map.data[i * width + j] = OccupancyGrid[i][j];
      }
    }

    costmap_pub_->publish(map);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
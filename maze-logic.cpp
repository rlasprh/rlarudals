#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
public:
  SelfDrive() : rclcpp::Node("self_drive")
  {
    auto lidar_qos = rclcpp::SensorDataQoS();
    
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      lidar_qos,
      std::bind(&SelfDrive::scan_callback, this, std::placeholders::_1)
    );

    
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel",
      rclcpp::QoS(10)
    );

    RCLCPP_INFO(get_logger(), "SelfDrive: Wide Front Check (Safety Mode)");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

  
  const double TARGET_SPEED = 0.15;
  const double ROTATE_SPEED = 1.0;   
  const double STOP_DIST = 0.25;     
  const double WALL_KEEP_DIST = 0.4; 

  
  double get_min_range(const sensor_msgs::msg::LaserScan::SharedPtr scan, double center_angle, int width_deg)
  {
    double angle_rad = center_angle * M_PI / 180.0;
    int center_idx = (angle_rad - scan->angle_min) / scan->angle_increment;
    int range_cnt = (width_deg * M_PI / 180.0) / scan->angle_increment;
    
    int start = center_idx - (range_cnt / 2);
    int end = center_idx + (range_cnt / 2);
    int total = scan->ranges.size();
    
    double min_val = 100.0; 

    for (int i = start; i <= end; ++i) {
      int idx = (i < 0) ? (i + total) : (i >= total ? i - total : i);
      float r = scan->ranges[idx];
      
      if (std::isfinite(r) && r > 0.0) {
        if (r < min_val) min_val = r;
      }
    }
    return min_val;
  }

  
  double get_average_range(const sensor_msgs::msg::LaserScan::SharedPtr scan, double center_angle, int width_deg)
  {
    double angle_rad = center_angle * M_PI / 180.0;
    int center_idx = (angle_rad - scan->angle_min) / scan->angle_increment;
    int range_cnt = (width_deg * M_PI / 180.0) / scan->angle_increment;
    
    int start = center_idx - (range_cnt / 2);
    int end = center_idx + (range_cnt / 2);
    int total = scan->ranges.size();
    
    double sum = 0.0;
    int count = 0;

    for (int i = start; i <= end; ++i) {
      int idx = (i < 0) ? (i + total) : (i >= total ? i - total : i);
      float r = scan->ranges[idx];
      
      if (std::isfinite(r) && r > 0.0) {
        if (r > WALL_KEEP_DIST) r = WALL_KEEP_DIST;
        sum += r;
        count++;
      }
    }
    return (count > 0) ? (sum / count) : WALL_KEEP_DIST;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

   
    double front_min = get_min_range(scan, 0.0, 60);
    
    double left_avg  = get_average_range(scan, 90.0, 30);
    double right_avg = get_average_range(scan, 270.0, 30);

   
    if (front_min < STOP_DIST)
    {
   
      cmd.twist.linear.x = 0.0;
      
      if (left_avg >= right_avg) {
        cmd.twist.angular.z = ROTATE_SPEED; 
      } else {
        cmd.twist.angular.z = -ROTATE_SPEED;
      }
    }
    else
    {
  
      cmd.twist.linear.x = TARGET_SPEED;
      
  
      double error = left_avg - right_avg;
      cmd.twist.angular.z = error * 0.5;
    }

    vel_pub_->publish(cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfDrive>());
  rclcpp::shutdown();
  return 0;
}



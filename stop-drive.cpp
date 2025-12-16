#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    // LiDAR standard QoS
    auto lidar_qos = rclcpp::SensorDataQoS();

    // Subscribe to LiDAR
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      lidar_qos,
      std::bind(&SelfDrive::scan_callback, this, std::placeholders::_1)
    );

    // Publish TwistStamped (TurtleBot3 requires this format)
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel",
      rclcpp::QoS(10)
    );

    RCLCPP_INFO(get_logger(), "SelfDrive (15cm stop, TwistStamped) node started!");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  int step_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    // --- Find valid minimum LiDAR distance ---
    float min_range = std::numeric_limits<float>::infinity();

    for (float r : scan->ranges)
    {
      if (r <= 0.0f) continue;          // ignore invalid 0 or negative
      if (!std::isfinite(r)) continue;  // ignore inf/nan
      if (r < min_range) min_range = r;
    }

    if (!std::isfinite(min_range))
    {
      RCLCPP_WARN(get_logger(),
        "[STEP %d] Warning: No valid LiDAR data. Keeping robot stopped.", step_);
      cmd.twist.linear.x = 0.0;
      vel_pub_->publish(cmd);
      step_++;
      return;
    }

    // --- STOP if obstacle closer than 18cm (0.18m) ---
    if (min_range < 0.18f)
    {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;

      RCLCPP_WARN(
        get_logger(),
        "[STEP %d] Obstacle detected at %.3f m (<18cm) → STOP!",
        step_, min_range
      );
    }
    else
    {
      // --- Auto Drive ---
      cmd.twist.linear.x = 0.1;   // forward motion
      cmd.twist.angular.z = 0.0;

      RCLCPP_INFO(
        get_logger(),
        "[STEP %d] Clear. min_range = %.3f m → Moving forward.",
        step_, min_range
      );
    }

    vel_pub_->publish(cmd);
    step_++;
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

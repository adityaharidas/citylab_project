#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&Patrol::patrol, this));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_ranges_ = msg->ranges;
    angle_min_ = msg->angle_min;
    angle_increment_ = msg->angle_increment;
  }

  void patrol() {
    if (laser_ranges_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Waiting for laser scan data...");
      return;
    }

    auto msg = geometry_msgs::msg::Twist();
    float min_safe_distance = 0.5;
    int num_rays = laser_ranges_.size();
    int center_index = num_rays / 2;
    int half_fov = center_index / 2;

    std::vector<float> front_ranges(
        laser_ranges_.begin() + center_index - half_fov,
        laser_ranges_.begin() + center_index + half_fov);

    float min_distance =
        *std::min_element(front_ranges.begin(), front_ranges.end());

    if (min_distance < min_safe_distance) {
      msg.linear.x = 0.2;
      msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Obstacle detected! Turning...");
    } else {
      msg.linear.x = 0.2;
      msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Moving forward...");
    }

    cmd_vel_pub_->publish(msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> laser_ranges_;
  float angle_min_, angle_increment_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

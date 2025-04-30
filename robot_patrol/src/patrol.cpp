#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <string>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node"), scan_info_done_(false), odom_info_done_(false)
    {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Patrol::odom_callback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::control_callback, this));

        RCLCPP_INFO(this->get_logger(), "Patrol node initialized");
    }

private:
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Laser scan
    bool scan_info_done_;
    float front_range_, left_range_, right_range_;
    int front_idx_from_, front_idx_to_;
    int left_idx_from_, left_idx_to_;
    int right_idx_from_, right_idx_to_;

    // Odometry
    bool odom_info_done_;
    double current_x_, current_y_, current_yaw_;
    double prev_x_, prev_y_, prev_yaw_;
    double distance_;

    float front_threshold_ = 0.35;  // 35 cm obstacle threshold

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!scan_info_done_) {
            float angle_increment = msg->angle_increment;
            int ranges_size = msg->ranges.size();

            front_idx_from_ = ranges_size / 2 - (M_PI / 6) / angle_increment;
            front_idx_to_   = ranges_size / 2 + (M_PI / 6) / angle_increment;

            right_idx_from_ = ranges_size / 2 - (M_PI / 2) / angle_increment;
            right_idx_to_   = front_idx_from_;

            left_idx_from_  = front_idx_to_;
            left_idx_to_    = ranges_size / 2 + (M_PI / 2) / angle_increment;

            scan_info_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Scan info initialized");
            return;
        }

        right_range_ = calculate_min_range(msg->ranges, right_idx_from_, right_idx_to_);
        front_range_ = calculate_min_range(msg->ranges, front_idx_from_, front_idx_to_);
        left_range_  = calculate_min_range(msg->ranges, left_idx_from_, left_idx_to_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        if (!odom_info_done_) {
            prev_x_ = current_x_;
            prev_y_ = current_y_;
            prev_yaw_ = current_yaw_;
            odom_info_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Odom initialized: x=%.2f, y=%.2f, yaw=%.2f", current_x_, current_y_, current_yaw_);
            return;
        }

        distance_ = calculate_distance(prev_x_, prev_y_, current_x_, current_y_);
        prev_x_ = current_x_;
        prev_y_ = current_y_;
        prev_yaw_ = current_yaw_;
    }

    void control_callback()
    {
        geometry_msgs::msg::Twist cmd_vel;

        if (!scan_info_done_) return;

        // Debug info
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Ranges (m) - Left: %.2f | Front: %.2f | Right: %.2f",
            left_range_, front_range_, right_range_);

        std::string safest_direction = "front";

        if (front_range_ < front_threshold_) {
            if (right_range_ > left_range_)
                safest_direction = "right";
            else
                safest_direction = "left";
        }

        if (safest_direction == "front") {
            cmd_vel.linear.x = 0.15;
            cmd_vel.angular.z = 0.0;
        } else if (safest_direction == "right") {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.5;
        } else if (safest_direction == "left") {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5;
        }

        velocity_publisher_->publish(cmd_vel);
    }

    float calculate_min_range(const std::vector<float> &ranges, int start, int end)
    {
        start = std::max(0, start);
        end = std::min(static_cast<int>(ranges.size()), end);

        std::vector<float> segment(ranges.begin() + start, ranges.begin() + end);
        segment.erase(std::remove_if(segment.begin(), segment.end(),
            [](float r) { return std::isnan(r) || std::isinf(r); }), segment.end());

        if (segment.empty()) return std::numeric_limits<float>::infinity();

        return *std::min_element(segment.begin(), segment.end());
    }

    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x2 - x1, y2 - y1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}

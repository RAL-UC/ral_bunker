#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class BaseControllerNode : public rclcpp::Node 
{
public:
  BaseControllerNode()
  : Node("base_controller_node")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Publisher for Pose2D messages (similar to your ROS1 logic)
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose2d", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1));

    // Timer for periodic control loop execution
    timer_ = this->create_wall_timer(
      100ms, std::bind(&BaseControllerNode::control_loop, this));

    // Set the desired base target position and heading
    target_x_ = 0.0;
    target_y_ = 1.0;
    target_theta_ = M_PI / 4.0;
  }

private:
  // Callback to receive odometry data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update current x and y positions
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Use tf2 to convert quaternion to roll, pitch, yaw
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Update current_theta_ with the computed yaw
    current_theta_ = yaw;
    
    // Create and publish a Pose2D message
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_x_;
    pose2d.y = current_y_;
    pose2d.theta = yaw;
    pose_pub_->publish(pose2d);
  }

  // Main control loop that computes errors and publishes Twist messages
  void control_loop() {
    // Compute distance error and desired heading toward the target
    double error_dist = std::sqrt(std::pow(target_x_ - current_x_, 2) +
                                  std::pow(target_y_ - current_y_, 2));
    double desired_heading = std::atan2(target_y_ - current_y_, target_x_ - current_x_);
    double error_theta = desired_heading - current_theta_;

    // Normalize heading error to [-pi, pi]
    error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

    // Simple P controllers (from the original controller: Kp_pos and Kp_head)
    double Kp_pos  = 0.5;
    double Kp_head = 2.0;

    double vel_lin_des = Kp_pos * error_dist;   // Desired linear velocity
    double vel_ang_des = Kp_head * error_theta;   // Desired angular velocity

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = vel_lin_des;
    twist_msg.angular.z = vel_ang_des;
    twist_pub_->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), "Error dist: %.4f, Error theta: %.4f", error_dist, error_theta);
  }

  // Node members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Target base position (set-point)
  double target_x_, target_y_, target_theta_;

  // Current base state (from odometry)
  double current_x_ = 0.0, current_y_ = 0.0, current_theta_ = 0.0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaseControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

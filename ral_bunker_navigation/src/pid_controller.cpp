#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

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
      "/odometry/filtered/local", 10,
      std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1));


    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/odometry/local", 10,
    //   std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1));

    // Subscriber for the planned path
    auto qos = rclcpp::QoS(10).transient_local();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", qos,
      std::bind(&BaseControllerNode::path_callback, this, std::placeholders::_1));
  }

private:

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
      // Store the received path in a member variable
      path_ = msg->poses;
      current_waypoint_index_ = 0;  // Reset to the beginning of the new path
      RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", path_.size());
      // Timer for periodic control loop execution
      timer_ = this->create_wall_timer(
        100ms, std::bind(&BaseControllerNode::control_loop, this));
    }
  }

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

  void control_loop() {
    if (!path_.empty() && current_waypoint_index_ < path_.size()) {
      target_x_ = path_[current_waypoint_index_].pose.position.x;
      target_y_ = path_[current_waypoint_index_].pose.position.y;
    }


    // Compute distance and heading errors toward the current target waypoint
    double error_dist = std::sqrt(std::pow(target_x_ - current_x_, 2) +
                                  std::pow(target_y_ - current_y_, 2));

    double min_dist_threshold = 0.1;  // Define a threshold for very close to the target
    double desired_heading;

    // When very close to the target, assume the desired heading is the current heading.
    // This makes the heading error zero.
    if (error_dist < min_dist_threshold) {
      desired_heading = current_theta_;
    } else {
      desired_heading = std::atan2(target_y_ - current_y_, target_x_ - current_x_);
    }

    double error_theta = desired_heading - current_theta_;
    error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

    RCLCPP_INFO(this->get_logger(), "Error distance: %.5f, Error heading: %.5f", error_dist, error_theta);

    // Define a threshold to determine when the waypoint is reached
    double waypoint_threshold = 0.1;  // in meters (adjust as needed)
    if (error_dist < waypoint_threshold && current_waypoint_index_ < path_.size() - 1) {
      current_waypoint_index_++;
      RCLCPP_INFO(this->get_logger(), "Reached waypoint, switching to waypoint index %zu", current_waypoint_index_);
    }

    // Assume dt is the control loop time step (100ms = 0.1 s)
    double dt = 0.1;
    
    // Update the integral errors
    error_dist_integral_ += error_dist * dt;
    error_theta_integral_ += error_theta * dt;

    // Compute the derivative errors
    double error_dist_derivative = (error_dist - error_dist_last_) / dt;
    double error_theta_derivative = (error_theta - error_theta_last_) / dt;

    // PID gains for linear (position) control
    double Kp_pos = 0.5;
    double Ki_pos = 0.0;
    double Kd_pos = 0.0;

    // PID gains for angular (heading) control
    double Kp_head = 2.0;
    double Ki_head = 0.0;
    double Kd_head = 0.0;

    // Compute the PID outputs for linear and angular velocities
    double vel_lin_des = Kp_pos * error_dist +
                        Ki_pos * error_dist_integral_ +
                        Kd_pos * error_dist_derivative;
    double vel_ang_des = Kp_head * error_theta +
                        Ki_head * error_theta_integral_ +
                        Kd_head * error_theta_derivative;

    // Save the current errors for the next cycle
    error_dist_last_ = error_dist;
    error_theta_last_ = error_theta;

    RCLCPP_INFO(this->get_logger(), "Desired linear velocity: %.5f, Desired angular velocity: %.5f", vel_lin_des, vel_ang_des);

    // Saturate the outputs if necessary
    if (vel_lin_des > 0.4) {
      vel_lin_des = 0.4;
    }
    else if (vel_lin_des < -0.4) {
      vel_lin_des = -0.4;
    }
    if (vel_ang_des > 0.3) {
      vel_ang_des = 0.3;
    }
    else if (vel_ang_des < -0.3) {
      vel_ang_des = -0.3;
    }

    // Publish the twist message
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = vel_lin_des;
    twist_msg.angular.z = vel_ang_des;
    twist_pub_->publish(twist_msg);
  }


  // Node members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Target base position (set-point)
  double target_x_, target_y_, target_theta_;

  // Current base state (from odometry)
  double current_x_ = 0.0, current_y_ = 0.0, current_theta_ = 0.0;

  // Storage for the received path and waypoint tracking
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_waypoint_index_ = 0;

  double error_dist_integral_ = 0.0;
  double error_dist_last_ = 0.0;
  double error_theta_integral_ = 0.0;
  double error_theta_last_ = 0.0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaseControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

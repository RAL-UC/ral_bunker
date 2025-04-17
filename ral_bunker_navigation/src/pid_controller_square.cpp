#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>  // for std::min, std::max

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

// Simple two-stage controller: ROTATE in place, then DRIVE with heading correction
enum class Stage { ROTATE, DRIVE };

class BaseControllerNode : public rclcpp::Node 
{
public:
  BaseControllerNode()
  : Node("pid_controller_square")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pose_pub_  = this->create_publisher<geometry_msgs::msg::Pose2D>("pose2d", 10);

    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/odometry/local", 10,
    //   std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered/local", 10,
      std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1));


    auto qos = rclcpp::QoS(10).transient_local();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", qos,
      std::bind(&BaseControllerNode::path_callback, this, std::placeholders::_1));

    // Initialize control stage and gains/thresholds
    stage_ = Stage::ROTATE;
    heading_thresh_ = 0.1;  // radians (~5°)
    dist_thresh_    = 0.1;  // meters
    k_ang_ = 1.0;
    k_lin_ = 0.3;
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
      path_ = msg->poses;
      current_waypoint_index_ = 0;
      stage_ = Stage::ROTATE;
      RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", path_.size());
      timer_ = this->create_wall_timer(
        100ms, std::bind(&BaseControllerNode::control_loop, this));
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_theta_ = yaw;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x     = current_x_;
    pose2d.y     = current_y_;
    pose2d.theta = current_theta_;
    pose_pub_->publish(pose2d);
  }

  void control_loop() {
    geometry_msgs::msg::Twist cmd;

    if (current_waypoint_index_ >= path_.size()) {
      // No more waypoints: stop and cancel
      twist_pub_->publish(cmd);
      timer_->cancel();
      return;
    }

    // Compute errors to the current waypoint
    auto &pt = path_[current_waypoint_index_].pose.position;
    double dx = pt.x - current_x_;
    double dy = pt.y - current_y_;
    double dist_err = std::hypot(dx, dy);
    double desired_yaw = std::atan2(dy, dx);
    double yaw_err = desired_yaw - current_theta_;
    yaw_err = std::atan2(std::sin(yaw_err), std::cos(yaw_err));  // wrap to [-π,π]

    RCLCPP_INFO(this->get_logger(), "Dist Error: %.3f, Yaw Error: %.3f", dist_err, yaw_err);

    if (stage_ == Stage::ROTATE) {
      // Rotate in place until aligned
      if (std::fabs(yaw_err) > heading_thresh_) {
        cmd.linear.x = 0.0;
        double w = k_ang_ * yaw_err;
        cmd.angular.z = std::max(-0.5, std::min(w, 0.5));
      } else {
        stage_ = Stage::DRIVE;
      }
    } else {
      // DRIVE stage: move forward *and* correct heading
      if (dist_err > dist_thresh_) {
        double v = k_lin_ * dist_err;
        cmd.linear.x = std::max(0.0, std::min(v, 0.4));
        double w = k_ang_ * yaw_err;
        cmd.angular.z = std::max(-0.5, std::min(w, 0.5));
      } else {
        // Reached corner: advance to next and rotate
        current_waypoint_index_++;
        stage_ = Stage::ROTATE;
      }
    }

    twist_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_waypoint_index_ = 0;

  Stage stage_;
  double heading_thresh_;
  double dist_thresh_;
  double k_ang_;
  double k_lin_;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_theta_ = 0.0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaseControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

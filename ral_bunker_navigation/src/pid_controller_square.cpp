#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>  // for std::min, std::max

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Action client
#include "rclcpp_action/rclcpp_action.hpp"
#include "radar_msg/action/next_pose.hpp"

using namespace std::chrono_literals;

// Simple two-stage controller: ROTATE in place, then DRIVE with heading correction
// Wait at each waypoint until action goal is received
enum class Stage { ROTATE, DRIVE };

class BaseControllerNode : public rclcpp::Node 
{
public:
  using NextPoseAction = radar_msg::action::NextPose;
  using GoalHandleNextPose = rclcpp_action::ServerGoalHandle<NextPoseAction>;

  BaseControllerNode()
  : Node("pid_controller_square")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pose_pub_  = this->create_publisher<geometry_msgs::msg::Pose2D>("pose2d", 10);

    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/odometry/filtered/local", 10,
    //   std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/local", 10,
      std::bind(&BaseControllerNode::odom_callback, this, std::placeholders::_1)); 

    auto qos = rclcpp::QoS(10).transient_local();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", qos,
      std::bind(&BaseControllerNode::path_callback, this, std::placeholders::_1));

    // Create action server
    this->action_server_ = rclcpp_action::create_server<NextPoseAction>(
      this,
      "next_pose",
      std::bind(&BaseControllerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BaseControllerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&BaseControllerNode::handle_accepted, this, std::placeholders::_1));

    // Initialize control stage and gains/thresholds
    stage_ = Stage::ROTATE;
    heading_thresh_ = 0.01;  // radians (~5°)
    dist_thresh_    = 0.02;  // meters
    k_ang_ = 1.0;
    k_lin_ = 0.4;
    executing_action_ = false;
    current_goal_handle_ = nullptr;
  }

private:
  rclcpp_action::Server<NextPoseAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleNextPose> current_goal_handle_;
  bool executing_action_;
  std::thread execution_thread_;  // Add this to track the execution thread

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NextPoseAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with go_to_next_pose = %s", goal->go_to_next_pose ? "true" : "false");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNextPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNextPose> goal_handle)
  {
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    execution_thread_ = std::thread{std::bind(&BaseControllerNode::execute, this, std::placeholders::_1), goal_handle};
    execution_thread_.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNextPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    // Store the goal handle for the control loop to use
    current_goal_handle_ = goal_handle;
    
    // If the goal is to go to next pose, set the flag
    if (goal_handle->get_goal()->go_to_next_pose) {
      executing_action_ = true;  // Allow proceeding to next waypoint
      stage_ = Stage::ROTATE; // Start with rotation
      RCLCPP_INFO(this->get_logger(), "Action goal received: proceeding to next waypoint");
    }

    // Main execution loop - keep running until action is complete
    rclcpp::Rate loop_rate(10);  // 10 Hz feedback rate
    
    while (rclcpp::ok() && executing_action_) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<NextPoseAction::Result>();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        executing_action_ = false;
        current_goal_handle_ = nullptr;
        return;
      }

      // Send feedback with distance to goal
      auto feedback = std::make_shared<NextPoseAction::Feedback>();
      
      // Calculate distance to goal
      if (current_waypoint_index_ < path_.size()) {
        auto &pt = path_[current_waypoint_index_].pose.position;
        double dx = pt.x - current_x_;
        double dy = pt.y - current_y_;
        feedback->distance_to_goal = std::hypot(dx, dy);
      } else {
        feedback->distance_to_goal = 0.0;
      }
      
      // Publish the feedback
      goal_handle->publish_feedback(feedback);
      
      // Sleep to maintain feedback rate
      loop_rate.sleep();
    }

    // Action is complete - send final result
    auto result = std::make_shared<NextPoseAction::Result>();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded - waypoint reached");
    
    // Clean up
    current_goal_handle_ = nullptr;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
      path_ = msg->poses;
      current_waypoint_index_ = 1;
      RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", path_.size());
      RCLCPP_INFO(this->get_logger(), "Path points:");
      for (size_t i = 0; i < path_.size(); ++i) {
        const auto &pose = path_[i].pose.position;
        const auto &orientation = path_[i].pose.orientation;
        RCLCPP_INFO(this->get_logger(), "  Waypoint %zu: (%.3f, %.3f, %.3f)", i, pose.x, pose.y, orientation.z);
      }
      // Start the control loop timer
      timer_ = this->create_wall_timer(
        100ms, std::bind(&BaseControllerNode::control_loop, this));
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received odometry update");
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
      executing_action_ = false;  // Signal action completion
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

    RCLCPP_INFO(this->get_logger(), "Dist Err: %.3f, Yaw Err: %.3f, Stage: %s, Executing action: %s", 
      dist_err, yaw_err,
      (stage_==Stage::ROTATE?"ROTATE":"DRIVE"),
      (executing_action_?"yes":"no"));

    if (!executing_action_) {
      // If not executing an action, just wait in place
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      twist_pub_->publish(cmd);
      return;
    }

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
      // DRIVE stage: move forward + heading correction, or wait at waypoint
      if (dist_err > dist_thresh_) {
        // still approaching
        double v = k_lin_ * dist_err;
        cmd.linear.x = std::max(0.0, std::min(v, 0.4));
        double w = k_ang_ * yaw_err;
        cmd.angular.z = std::max(-0.5, std::min(w, 0.5));
      } else {
        // Reached waypoint - action is complete
        executing_action_ = false;  // This will signal the execute function to finish
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu, action complete", current_waypoint_index_);

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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PathPublisherNode : public rclcpp::Node
{
public:
  PathPublisherNode()
  : Node("path_publisher_node")
  {
    // Declare the "path_type" parameter with a default value "line"
    this->declare_parameter<std::string>("path_type", "line");

    // Create a publisher for nav_msgs/Path messages on the "plan" topic
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(10).transient_local());    

    // Create a one-shot timer to publish the path after a short delay
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PathPublisherNode::publish_path, this));
  }

private:
  // Function to create a straight line path
  nav_msgs::msg::Path createStraightLinePath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "odom";

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    int num_points = 6;  // For example, 6 waypoints
    for (int i = 1; i < num_points; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->get_clock()->now();
      pose.header.frame_id = "odom";

      pose.pose.position.x = i * 1.0;  // 1 meter spacing along x
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;

      // No rotation: quaternion for 0 rad about Z-axis
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      poses.push_back(pose);
    }
    path_msg.poses = poses;
    return path_msg;
  }

  // Function to create a snake path (an oscillating path along x)
  nav_msgs::msg::Path createSnakePath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "odom";

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    int num_points = 8;           // Number of waypoints in the snake
    double spacing = 1.5;          // Spacing between waypoints in x
    double amplitude = 1.2;        // Amplitude of the sine wave
    double omega = 0.5;            // Frequency multiplier for the sine wave

    for (int i = 1; i < num_points; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->get_clock()->now();
      pose.header.frame_id = "odom";

      // x increases linearly
      pose.pose.position.x = i * spacing;
      // y oscillates in a sine wave
      pose.pose.position.y = amplitude * std::sin(i * omega);
      pose.pose.position.z = 0.0;

      // Compute a heading that roughly follows the path's tangent:
      // The derivative of y = A*sin(omega*x) is A*omega*cos(omega*x).
      double derivative = amplitude * omega * std::cos(i * omega);
      double heading = std::atan2(derivative, 1.0);

      // Convert heading to a quaternion (rotation about z)
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(heading / 2.0);
      pose.pose.orientation.w = std::cos(heading / 2.0);

      poses.push_back(pose);
    }
    path_msg.poses = poses;
    return path_msg;
  }

  // Function to create a square path
  nav_msgs::msg::Path createSquarePath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "odom";

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    // Define square side length and corners (return to start)
    double side = 2.0;
    std::vector<std::pair<double, double>> corners = {
      {0.0, 0.0},
      {side, 0.0},
      {side, side},
      {0.0, side},
      {0.0, 0.0}
    };
    // Create poses for each corner
    for (size_t i = 0; i < corners.size(); ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->get_clock()->now();
      pose.header.frame_id = "odom";

      // Current corner
      double x = corners[i].first;
      double y = corners[i].second;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;

      // Compute heading toward next corner
      size_t next = (i + 1) % corners.size();
      double dx = corners[next].first - x;
      double dy = corners[next].second - y;
      double heading = std::atan2(dy, dx);

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(heading / 2.0);
      pose.pose.orientation.w = std::cos(heading / 2.0);

      poses.push_back(pose);
    }
    path_msg.poses = poses;
    return path_msg;
  }

  // Publish the path based on the "path_type" parameter
  void publish_path()
  {
    // Retrieve the parameter value
    std::string path_type;
    this->get_parameter("path_type", path_type);

    nav_msgs::msg::Path path_msg;
    if (path_type == "line")
    {
      RCLCPP_INFO(this->get_logger(), "Publishing straight line path");
      path_msg = createStraightLinePath();
    }
    else if (path_type == "snake")
    {
      RCLCPP_INFO(this->get_logger(), "Publishing snake path");
      path_msg = createSnakePath();
    }
    else if (path_type == "square")
    {
      RCLCPP_INFO(this->get_logger(), "Publishing square path");
      path_msg = createSquarePath();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
        "Unknown path_type parameter: '%s'. Defaulting to straight line", path_type.c_str());
      path_msg = createStraightLinePath();
    }

    // Publish the path message
    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_msg.poses.size());

    // Cancel the timer so we only publish once
    timer_->cancel();
  }

  // Node members
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisherNode>();
    rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

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

  nav_msgs::msg::Path createSquarePath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "odom";

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    double side = 6.0;
    const int num_intermediate = 3;

    // 1) Define the four unique corners
    std::vector<std::pair<double,double>> corners = {
      {0.0, 0.0},
      { side, 0.0},
      { side,  side},
      {0.0,  side}
    };

    // 2) Create and push the starting pose (at 0,0, heading=0)
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "odom";
    start_pose.header.stamp = this->get_clock()->now();
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;
    // heading = 0 → quaternion = (0,0,0,1)
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.0;
    start_pose.pose.orientation.w = 1.0;
    poses.push_back(start_pose);

    // 3) Walk each edge, including intermediate points, and push the corner
    for (size_t i = 0; i < corners.size(); ++i)
    {
      // next corner index wraps around to 0
      size_t j = (i + 1) % corners.size();
      double x0 = corners[i].first,  y0 = corners[i].second;
      double x1 = corners[j].first,  y1 = corners[j].second;

      double dx = x1 - x0;
      double dy = y1 - y0;
      double heading = std::atan2(dy, dx);
      // build quaternion for this segment’s heading
      double qz = std::sin(heading / 2.0);
      double qw = std::cos(heading / 2.0);

      // intermediate points (split edge into num_intermediate+1 segments)
      for (int k = 1; k <= num_intermediate; ++k)
      {
        double t = double(k) / double(num_intermediate + 1);
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "odom";
        p.header.stamp = this->get_clock()->now();
        p.pose.position.x = x0 + t * dx;
        p.pose.position.y = y0 + t * dy;
        p.pose.position.z = 0.0;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = qz;
        p.pose.orientation.w = qw;
        poses.push_back(p);
      }

      // then push the actual corner
      geometry_msgs::msg::PoseStamped corner_pose;
      corner_pose.header.frame_id = "odom";
      corner_pose.header.stamp = this->get_clock()->now();
      corner_pose.pose.position.x = x1;
      corner_pose.pose.position.y = y1;
      corner_pose.pose.position.z = 0.0;
      corner_pose.pose.orientation.x = 0.0;
      corner_pose.pose.orientation.y = 0.0;
      corner_pose.pose.orientation.z = qz;
      corner_pose.pose.orientation.w = qw;
      poses.push_back(corner_pose);
    }

    // The last corner pushed is back at (0,0), so path is closed.
    poses.push_back(start_pose);
    path_msg.poses = std::move(poses);
    return path_msg;
  }

  nav_msgs::msg::Path createRectanglePath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "odom";

    const double length = 4.5;        // X-extent
    const double width  = 4.5;        // Y-extent
    const int    pts_len = 4;
    const int    pts_wid = 4;

    std::vector<geometry_msgs::msg::PoseStamped> poses;

    const std::vector<std::pair<double,double>> corners = {
        {0.0,    0.0},
        {length, 0.0},
        {length, width},
        {0.0,    width}
    };

    auto push_pose = [&](double x, double y, double heading)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = this->get_clock()->now();
      p.header.frame_id = "odom";
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.position.z = 0.0;
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;
      p.pose.orientation.z = std::sin(heading/2.0);
      p.pose.orientation.w = std::cos(heading/2.0);
      poses.push_back(std::move(p));
    };

    push_pose(0.0, 0.0, 0.0);   // start pose

    /*  walk edges, sprinkling intermediate points  */
    for (size_t i = 0; i < corners.size(); ++i)
    {
      size_t j = (i + 1) % corners.size();
      double x0 = corners[i].first,  y0 = corners[i].second;
      double x1 = corners[j].first,  y1 = corners[j].second;

      double dx = x1 - x0;
      double dy = y1 - y0;
      double heading = std::atan2(dy, dx);

      /* choose how many intermediate points for this edge */
      int n_int = (std::fabs(dx) > std::fabs(dy)) ? pts_len : pts_wid;

      for (int k = 1; k <= n_int; ++k)
      {
        double t = static_cast<double>(k) / (n_int + 1);
        push_pose(x0 + t*dx, y0 + t*dy, heading);
      }

      if (j != 0)   // avoid duplicating the start corner at the very end
        push_pose(x1, y1, heading);
    }

    push_pose(0.0, 0.0, 0.0);   // close the loop
    path_msg.poses = std::move(poses);
    return path_msg;
  }

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
    else if (path_type == "rectangle")
    {
      RCLCPP_INFO(this->get_logger(), "Publishing rectangle path");
      path_msg = createRectanglePath();
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

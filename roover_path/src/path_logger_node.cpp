#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathLoggerNode : public rclcpp::Node {
public:
  PathLoggerNode() : Node("path_logger_node") {
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&PathLoggerNode::odom_callback, this, std::placeholders::_1)
    );

    path_.header.frame_id = "odom";
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    path_.header.stamp = msg->header.stamp;
    path_.poses.push_back(pose);

    path_pub_->publish(path_);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathLoggerNode>());
  rclcpp::shutdown();
  return 0;
}

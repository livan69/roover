#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GPSPathNode : public rclcpp::Node
{
public:
    GPSPathNode() : Node("gps_path_node")
    {
        path_.header.frame_id = "map";

        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gps/filtered", 10,
            std::bind(&GPSPathNode::odom_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Path>("/gps/path", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        path_.header.stamp = this->get_clock()->now();
        path_.poses.push_back(pose);

        pub_->publish(path_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSPathNode>());
    rclcpp::shutdown();
    return 0;
}

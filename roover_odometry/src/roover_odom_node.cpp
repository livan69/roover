#include "rclcpp/rclcpp.hpp"
#include "roover_serial_cpp/msg/roover_feedback.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class RooverOdomNode : public rclcpp::Node {
public:
  RooverOdomNode() : Node("roover_odometry"), x_(0.0), y_(0.0), theta_(0.0) {
    // Params
    declare_parameter("wheel_base", 0.54);   // afstand tussen de wielen (m)
    declare_parameter("ticks_per_rev", 90);
    declare_parameter("wheel_diameter", 0.165);

    wheel_base_     = get_parameter("wheel_base").as_double();
    ticks_per_rev_  = get_parameter("ticks_per_rev").as_int();
    wheel_diam_     = get_parameter("wheel_diameter").as_double();

    dist_per_tick_ = (M_PI * wheel_diam_) / ticks_per_rev_;

    // Subscriber
    sub_ = create_subscription<roover_serial_cpp::msg::RooverFeedback>(
      "roover_feedback", 10,
      std::bind(&RooverOdomNode::feedback_cb, this, std::placeholders::_1)
    );

    // Publisher
    pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Init
    last_pub_time_ = now();


    // Log melding bij opstart
    RCLCPP_INFO(this->get_logger(), "Roover odometry node gestart (10 Hz throttle)!");
  }

private:
  void feedback_cb(const roover_serial_cpp::msg::RooverFeedback::SharedPtr msg) {
    rclcpp::Time now_time = now();
    
    // throttle naar 10 Hz
    if ((now_time - last_pub_time_).seconds() < (1.0 / publish_rate_)) {
      return;
    }
    last_pub_time_ = now_time;

    int wrap_max = 9000;
    // Bereken delta encoder
    auto delta_mod = [&](int curr, int prev) {
      int d = curr - prev;
      if (d >  wrap_max/2) d -= wrap_max;   // overflow vooruit
      if (d < -wrap_max/2) d += wrap_max;   // overflow achteruit
      return d;
    };

    int dL = delta_mod(msg->wheel_l_cnt, prev_left_);
    int dR = delta_mod(msg->wheel_r_cnt, prev_right_);

    prev_left_  = msg->wheel_l_cnt;
    prev_right_ = msg->wheel_r_cnt;

    // int dL = msg->wheel_l_cnt - prev_left_;
    // int dR = msg->wheel_r_cnt - prev_right_;
    // prev_left_ = msg->wheel_l_cnt;
    // prev_right_ = msg->wheel_r_cnt;

    // Afstand per wiel
    double dl = dL * dist_per_tick_;
    double dr = dR * dist_per_tick_;
    double dc = (dl + dr) / 2.0;
    double dtheta = (dr - dl) / wheel_base_;

    // Update pose
    x_     += dc * cos(theta_ + dtheta/2.0);
    y_     += dc * sin(theta_ + dtheta/2.0);
    theta_ += dtheta;

    // Maak Odometry msg
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = sin(theta_/2.0);
    odom.pose.pose.orientation.w = cos(theta_/2.0);

    pub_->publish(odom);

    // TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  // Params
  double wheel_base_;
  int ticks_per_rev_;
  double wheel_diam_;
  double dist_per_tick_;

  // Pose state
  double x_, y_, theta_;
  int prev_left_ = 0, prev_right_ = 0;

  // Timing
  rclcpp::Time last_pub_time_;
  double publish_rate_ = 10.0; // Hz

  rclcpp::Subscription<roover_serial_cpp::msg::RooverFeedback>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RooverOdomNode>());
  rclcpp::shutdown();
  return 0;
}

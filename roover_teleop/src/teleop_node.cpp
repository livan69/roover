
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "roover_f710_input/msg/f710_input.hpp"

using std::placeholders::_1;
using F710Input = roover_f710_input::msg::F710Input;

const int DEADZONE = 800;

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop_node")
  {
    // Publisher op cmd_vel
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10)
    );

    // Subscriber op joystick
    sub_ = this->create_subscription<F710Input>(
      "f710_input", rclcpp::QoS(10),
      std::bind(&TeleopNode::joy_cb, this, _1)
    );
  }

private:
  void joy_cb(const F710Input::SharedPtr msg)
  {
    auto twist = geometry_msgs::msg::Twist();

    // Map [-32767,32767] → [-1.0,1.0]
    auto normalize = [](int val) {
      if (std::abs(val) < DEADZONE) return 0.0;
      return static_cast<double>(val) / 32767.0;
    };

    double lx = normalize(msg->right_x);
    double ly = -normalize(msg->left_y);

    // Snelheid en draaiing
    twist.linear.x  =  ly * 0.2;       // vooruit/achteruit trager zetten , ging veel te snel
    twist.angular.z = lx * 0.3;       // links/rechts

    pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<F710Input>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopNode>());
  rclcpp::shutdown();
  return 0;
}

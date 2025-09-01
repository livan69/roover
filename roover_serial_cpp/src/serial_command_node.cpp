// src/serial_command_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "roover_serial_cpp/serial_command.hpp"  // jouw header met SerialCommand
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

using Twist = geometry_msgs::msg::Twist;
const int DEADZONE = 5;


class SerialCommandNode : public rclcpp::Node
{
public:
  SerialCommandNode()
  : Node("serial_command_node")
  {
    this->declare_parameter<std::string>("device", "/dev/ttyAMA0");
    this->declare_parameter<int>("baudrate", 115200);
    get_parameter("device", dev_);
    get_parameter("baudrate", baud_);

    // Open UART
    fd_ = open(dev_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Kan seriële poort niet openen: %s", dev_.c_str());
      rclcpp::shutdown();
      return;
    }
    termios tty{};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_);
    cfsetospeed(&tty, baud_);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tty);

    // Subscriber op Twist in plaats van F710Input
    sub_ = create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&SerialCommandNode::twist_cb, this, std::placeholders::_1)
    );
  }

  ~SerialCommandNode() override
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  void twist_cb(const Twist::SharedPtr msg)
  {
    // Map linear.x [-1.0,1.0] → speed [-1000,1000]
    int16_t speed = static_cast<int16_t>(msg->linear.x  * 1000.0);
    // Map angular.z [-1.0,1.0] → steer [-1000,1000]
    int16_t steer = static_cast<int16_t>(msg->angular.z * 1000.0);

    SerialCommand cmd{};
    cmd.start    = SERIAL_START_FRAME;

    if (abs(steer) <= DEADZONE) steer= 0.0;
    if (abs(speed) <= DEADZONE) speed= 0.0;

    cmd.steer    = steer;
    cmd.speed    = speed;
    cmd.checksum = cmd.start ^ cmd.steer ^ cmd.speed;

    write(fd_, &cmd, sizeof(cmd));
    usleep(2000);  // 2 ms gap om een idle-line op de GD32 uit te lokken
  }

  std::string dev_;
  int         baud_;
  int         fd_{-1};
  rclcpp::Subscription<Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

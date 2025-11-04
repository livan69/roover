#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>

class LidarNode : public rclcpp::Node {
public:
  LidarNode() : Node("lidar_node"), fd_(-1) {
    declare_parameter("device", "/dev/lidar");
    declare_parameter("baudrate", 230400);
    get_parameter("device", device_);
    baudrate_ = get_parameter("baudrate").as_int();

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LidarNode::poll_device, this)
    );

    RCLCPP_INFO(get_logger(), "Delta-2A LiDAR node gestart (%s @ %d)",
                device_.c_str(), baudrate_);
  }

private:
  void poll_device() {
    if (fd_ < 0) {
      try_open();
      return;
    }

    uint8_t buf[1024];
    int n = read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Geen data van LiDAR, probeer opnieuw te verbinden…");
      close(fd_);
      fd_ = -1;
      return;
    }

    // Hier komt later parsing van Delta-2A frames (placeholder)
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = now();
    msg.header.frame_id = "laser_frame";
    msg.angle_min = -3.14;
    msg.angle_max = 3.14;
    msg.angle_increment = 0.0174;
    msg.range_min = 0.1;
    msg.range_max = 12.0;
    msg.ranges.resize(360, 0.0);

    pub_->publish(msg);
  }

  void try_open() {
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Kan LiDAR niet openen op %s", device_.c_str());
      return;
    }

    struct termios tty{};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tty);

    RCLCPP_INFO(get_logger(), "LiDAR verbonden op %s", device_.c_str());
  }

  std::string device_;
  int baudrate_;
  int fd_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}

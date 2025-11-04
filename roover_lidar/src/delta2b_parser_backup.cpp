#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

class Delta2BNode : public rclcpp::Node {
public:
  Delta2BNode() : Node("delta2b_lidar_node"), fd_(-1) {
    declare_parameter("device", "/dev/ttyUSB1");
    declare_parameter("baudrate", 230400);
    get_parameter("device", device_);
    baudrate_ = get_parameter("baudrate").as_int();

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&Delta2BNode::poll, this));

    RCLCPP_INFO(get_logger(), "Delta-2B LiDAR gestart op %s @ %d",
                device_.c_str(), baudrate_);
    open_port();
  }

private:
  void open_port() {
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Kan poort %s niet openen", device_.c_str());
      return;
    }

    struct termios tty{};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tty);
  }

  void poll() {
    if (fd_ < 0) return;

    uint8_t buf[2048];
    int n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return;

    // Zoek frames die starten met 0xAA 0x55
    for (int i = 0; i < n - 22; i++) {
      if (buf[i] == 0xAA && buf[i + 1] == 0x55) {
        parse_frame(&buf[i]);
        break;
      }
    }
  }

  void parse_frame(uint8_t *data) {
    // Eenvoudige parser: 12 punten per frame (2 bytes per afstand)
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = "laser_frame";
    msg.angle_min = -3.14;
    msg.angle_max = 3.14;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / 360.0;
    msg.range_min = 0.1;
    msg.range_max = 12.0;
    msg.ranges.resize(360, 0.0);

    for (int j = 0; j < 12; j++) {
      uint16_t raw = (uint16_t)data[4 + j * 2] | ((uint16_t)data[5 + j * 2] << 8);
      float dist = raw / 1000.0; // in meter
      int idx = j * 30;
      if (idx < 360) msg.ranges[idx] = dist;
    }

    pub_->publish(msg);
  }

  std::string device_;
  int baudrate_;
  int fd_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Delta2BNode>());
  rclcpp::shutdown();
  return 0;
}

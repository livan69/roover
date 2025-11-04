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
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&Delta2BNode::poll, this));

    RCLCPP_INFO(get_logger(), "Delta-2B LiDAR gestart op %s @ %d", device_.c_str(), baudrate_);
  }

private:
  void poll() {
    if (fd_ < 0) {
      try_open();
      return;
    }

    uint8_t buf[512];
    int n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return;

    for (int i = 0; i < n - 2; i++) {
      if (buf[i] == 0xAA && buf[i + 1] == 0x00) {
        if (i + 146 <= n) {
          parse_frame(&buf[i]);
          break;
        }
      }
    }
  }

  void parse_frame(const uint8_t *frame) {
    const int data_offset = 10;      // start data in frame
    const int points_in_frame = 12;  // aantal metingen per frame

    // Hoekinfo uit header
    uint16_t raw_angle = frame[2] | (frame[3] << 8);
    float start_angle_deg = raw_angle / 64.0f;  // uit Delta-2A SDK

    // Volgende framehoek voor interpolatie (ruwe inschatting)
    uint16_t next_raw_angle = frame[2 + 90] | (frame[3 + 90] << 8);  // 90 bytes verder ≈ volgende frame
    float end_angle_deg = next_raw_angle / 64.0f;

    if (end_angle_deg < start_angle_deg)
        end_angle_deg += 360.0f;

    float angle_increment = (end_angle_deg - start_angle_deg) / points_in_frame;

    for (int i = 0; i < points_in_frame; i++) {
        int idx = data_offset + i * 2;
        uint16_t dist_raw = frame[idx] | (frame[idx + 1] << 8);
        float dist_m = static_cast<float>(dist_raw) / 10000.0f;  // 0.1 mm → m

        float angle_deg = start_angle_deg + i * angle_increment;
        if (angle_deg >= 360.0f) angle_deg -= 360.0f;
        size_t angle_bin = static_cast<size_t>(angle_deg);

        if (angle_bin < 360) {
        if (dist_m >= 0.1f && dist_m <= 8.0f)
            ranges_[angle_bin] = dist_m;
        else
            ranges_[angle_bin] = std::numeric_limits<float>::infinity();
        }
    }

    frame_index_++;
    if (frame_index_ >= 30) {  // ≈ 360°
        publish_scan();
        frame_index_ = 0;
    }
  }

  void publish_scan() {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = "laser_frame";
    msg.angle_min = -3.14;
    msg.angle_max = 3.14;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / 360.0;
    msg.range_min = 0.1;
    msg.range_max = 8.0;
    msg.ranges = ranges_;

    pub_->publish(msg);
    std::fill(ranges_.begin(), ranges_.end(), 0.0f);
  }

  void try_open() {
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_WARN(get_logger(), "Kan LiDAR niet openen op %s", device_.c_str());
      return;
    }

    struct termios tty {};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tty);

    RCLCPP_INFO(get_logger(), "Seriële poort geopend op %s", device_.c_str());
  }

  std::string device_;
  int baudrate_;
  int fd_;
  int frame_index_ = 0;
  std::vector<float> ranges_ = std::vector<float>(360, 0.0f);
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Delta2BNode>());
  rclcpp::shutdown();
  return 0;
}

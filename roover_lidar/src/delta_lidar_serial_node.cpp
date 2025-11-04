#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <cstring>
#include <limits>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

class DeltaLidarNode : public rclcpp::Node {
public:
  DeltaLidarNode() : Node("delta_lidar_serial_node"), fd_(-1), last_angle_(0.0f) {
    declare_parameter("port", "/dev/ttyUSB1");
    declare_parameter("baudrate", 230400);
    declare_parameter("frame_id", "laser_frame");

    get_parameter("port", port_);
    baudrate_ = get_parameter("baudrate").as_int();
    get_parameter("frame_id", frame_id_);

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    open_port();
    RCLCPP_INFO(get_logger(), "Liedje gestart op %s @ %d", port_.c_str(), baudrate_);

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&DeltaLidarNode::poll_serial, this));
  }

private:
  void open_port() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Kan poort %s niet openen", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    struct termios tty {};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tty);

    RCLCPP_INFO(get_logger(), "Seriële paart geopend: %s", port_.c_str());
  }

  void poll_serial() {
    if (fd_ < 0) return;

    uint8_t buf[512];
    int n = read(fd_, buf, sizeof(buf));
    n = read(fd_, buf, sizeof(buf));
    RCLCPP_DEBUG(this->get_logger(), "Bytes gelezen: %d", n);
    if (n <= 0) return;

    for (int i = 0; i < n - 10; i++) {
      RCLCPP_DEBUG(this->get_logger(), "Byte %d: 0x%02X 0x%02X", i, buf[i], buf[i+1]);
      if (buf[i] == 0xAA && buf[i + 1] == 0x00) {
        uint16_t payload_len = buf[i + 6] | (buf[i + 7] << 8);
        int frame_len = payload_len + 10; // 8 header + 2 CRC
        RCLCPP_INFO(this->get_logger(),
          "🔍 Found header at %d → payload_len=%u → frame_len=%d → n=%d",
          i, payload_len, frame_len, n);
        if (i + frame_len <= n) {
          RCLCPP_INFO(this->get_logger(), "Valid frame gevonden, len=%d", frame_len);
          parse_frame(&buf[i], frame_len);
          i += frame_len - 1;
        }
      }
    }
  }

  void parse_frame(const uint8_t *frame, int length) {
    //if (!frame || length < 16) return;

    if (frame[0] != 0xAA || frame[1] != 0x00)
      return;

    uint16_t payload_len = frame[6] | (frame[7] << 8);
    if (payload_len + 10 > length)
      return;

    const uint8_t *payload = frame + 8;

    // Header van payload
    //uint8_t lidar_speed = payload[0];                         // niet gebruikt
    //int16_t offset_angle = (payload[1] | (payload[2] << 8));   // niet gebruikt
    uint16_t start_angle_raw = (payload[3] | (payload[4] << 8));
    float start_angle = start_angle_raw * 0.01f;

    int num_points = (payload_len - 5) / 3;
    if (num_points <= 0) return;

    // Schatting van end-angle uit volgende frames vervangen door vaste stap
    float step = 360.0f / 1440.0f; // ±0.25° per punt bij 12*120 frames/s

    for (int i = 0; i < num_points; i++) {
      uint16_t dist_raw = payload[5 + 3 * i + 1] | (payload[5 + 3 * i + 2] << 8);
      float dist_m = dist_raw / 4.0f / 1000.0f;

      float angle = fmod(start_angle + i * step, 360.0f);
      int angle_i = static_cast<int>(angle);
      if (angle_i < 0 || angle_i >= 360) continue;

      if (dist_m < 0.1f || dist_m > 8.0f)
        ranges_[angle_i] = std::numeric_limits<float>::infinity();
      else
        ranges_[angle_i] = dist_m;
    }

    // Nieuwe scan bij hoekwrap
    if (start_angle < last_angle_) {
      publish_scan();
      std::fill(ranges_.begin(), ranges_.end(),
                std::numeric_limits<float>::infinity());
    }
    last_angle_ = start_angle;
  }

  void publish_scan() {
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.angle_min = DEG2RAD(0.0);
    msg.angle_max = DEG2RAD(359.0);
    msg.angle_increment = DEG2RAD(1.0);
    msg.range_min = 0.1;
    msg.range_max = 8.0;
    msg.ranges = ranges_;

    pub_->publish(msg);
  }

  // === leden ===
  std::string port_;
  std::string frame_id_;
  int baudrate_;
  int fd_;
  float last_angle_;
  std::vector<float> ranges_ = std::vector<float>(360, std::numeric_limits<float>::infinity());
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

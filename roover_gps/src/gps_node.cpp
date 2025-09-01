#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/string.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <cmath>

class GPSNode : public rclcpp::Node {
public:
    GPSNode() : Node("gps_node"), fd_(-1) {
        declare_parameter("device", "/dev/ttyUSB0");
        declare_parameter("baudrate", 115200);

        get_parameter("device", device_);
        baudrate_ = get_parameter("baudrate").as_int();

        // Seriële poort openen
        fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Kon GPS seriële poort niet openen: %s", device_.c_str());
            rclcpp::shutdown();
        }

        struct termios tty{};
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tcsetattr(fd_, TCSANOW, &tty);

        pub_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
        pub_raw_ = create_publisher<std_msgs::msg::String>("nmea_raw", 10);

        // Start aparte reader thread
        reader_thread_ = std::thread([this]() { this->reader_loop(); });

        RCLCPP_INFO(get_logger(), "GPS node gestart op %s @ %d baud", device_.c_str(), baudrate_);
    }

    ~GPSNode() {
        if (reader_thread_.joinable()) reader_thread_.join();
        if (fd_ >= 0) close(fd_);
    }

private:
    void reader_loop() {
        char buf[256];
        while (rclcpp::ok()) {
            int n = read(fd_, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                buffer_ += buf;

                size_t pos;
                while ((pos = buffer_.find("\n")) != std::string::npos) {
                    std::string line = buffer_.substr(0, pos);
                    buffer_.erase(0, pos + 1);

                    // publiceer raw debug
                    auto raw_msg = std_msgs::msg::String();
                    raw_msg.data = line;
                    pub_raw_->publish(raw_msg);

                    if (line.rfind("$GNGGA", 0) == 0) {
                        parse_gga(line);
                    } else if (line.rfind("$GNRMC", 0) == 0) {
                        RCLCPP_DEBUG(get_logger(), "RMC: %s", line.c_str());
                    }
                }
            }
        }
    }

    void parse_gga(const std::string &sentence) {
        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string item;
        while (std::getline(ss, item, ',')) fields.push_back(item);

        if (fields.size() < 10) return;

        double lat = nmea_to_deg(fields[2], fields[3]);
        double lon = nmea_to_deg(fields[4], fields[5]);

        double alt = 0.0;
        if (fields.size() > 9 && !fields[9].empty()) {
            alt = std::stod(fields[9]);
        }

        auto msg = sensor_msgs::msg::NavSatFix();
        msg.header.stamp = now();
        msg.header.frame_id = "gps_link";
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg.latitude = lat;
        msg.longitude = lon;
        msg.altitude = alt;

        pub_fix_->publish(msg);

        //RCLCPP_INFO(get_logger(), "GPS Fix: lat=%.6f lon=%.6f alt=%.2f",
        //            lat, lon, alt);
    }

    double nmea_to_deg(const std::string &val, const std::string &hemisphere) {
        if (val.empty()) return 0.0;
        double raw = std::stod(val);
        double deg = floor(raw / 100.0);
        double min = raw - (deg * 100.0);
        double dec = deg + (min / 60.0);
        if (hemisphere == "S" || hemisphere == "W") dec = -dec;
        return dec;
    }

    std::string device_;
    int baudrate_;
    int fd_;
    std::string buffer_;
    std::thread reader_thread_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_raw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNode>());
    rclcpp::shutdown();
    return 0;
}

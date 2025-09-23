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
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

class GPSNode : public rclcpp::Node {
public:
    GPSNode() : Node("gps_node"), fd_(-1), last_fix_quality_(-1) {
        declare_parameter("device", "/dev/ttyUSB0");
        declare_parameter("baudrate", 115200);
        declare_parameter("rtcm_host", "192.168.0.32");
        declare_parameter("rtcm_port", 5016);

        get_parameter("device", device_);
        baudrate_ = get_parameter("baudrate").as_int();
        rtcm_host_ = get_parameter("rtcm_host").as_string();
        rtcm_port_ = get_parameter("rtcm_port").as_int();

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

        // Start aparte threads
        reader_thread_ = std::thread([this]() { this->reader_loop(); });
        rtcm_thread_   = std::thread([this]() { this->rtcm_loop(); });

        RCLCPP_INFO(get_logger(), "GPS node gestart op %s @ %d baud (RTCM host=%s:%d)",
                    device_.c_str(), baudrate_, rtcm_host_.c_str(), rtcm_port_);
    }

    ~GPSNode() {
        if (reader_thread_.joinable()) reader_thread_.join();
        if (rtcm_thread_.joinable()) rtcm_thread_.join();
        if (fd_ >= 0) close(fd_);
    }

private:
    // Thread: lees NMEA uit GPS
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

    // Thread: verbind met caster en stuur RTCM naar GPS
    void rtcm_loop() {
        while (rclcpp::ok()) {
            int sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                RCLCPP_ERROR(get_logger(), "Kon socket niet maken");
                sleep(5);
                continue;
            }

            struct sockaddr_in serv_addr{};
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(rtcm_port_);
            if (inet_pton(AF_INET, rtcm_host_.c_str(), &serv_addr.sin_addr) <= 0) {
                RCLCPP_ERROR(get_logger(), "Ongeldig RTK hostadres");
                close(sock);
                sleep(5);
                continue;
            }

            RCLCPP_INFO(get_logger(), "Verbinden met RTCM caster %s:%d...",
                        rtcm_host_.c_str(), rtcm_port_);
            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                RCLCPP_WARN(get_logger(), "Caster connectie mislukt, opnieuw proberen...");
                close(sock);
                sleep(5);
                continue;
            }

            RCLCPP_INFO(get_logger(), "RTCM connectie actief");

            char buf[1024];
            while (rclcpp::ok()) {
                int n = read(sock, buf, sizeof(buf));
                if (n <= 0) {
                    RCLCPP_WARN(get_logger(), "Caster connectie verbroken, opnieuw proberen...");
                    break;
                }

                // ✅ Debug: log aantal bytes en eerste paar hex
                std::stringstream hex;
                for (int i = 0; i < std::min(n, 16); i++) {
                    hex << std::hex << std::uppercase
                        << std::setw(2) << std::setfill('0')
                        << (static_cast<int>(static_cast<unsigned char>(buf[i]))) << " ";
                }
                // RCLCPP_INFO(get_logger(),
                //             "RTCM ontvangen: %d bytes [hex: %s...]",
                //             n, hex.str().c_str());

                if (fd_ >= 0) {
                    int w = write(fd_, buf, n);
                    RCLCPP_DEBUG(get_logger(), "RTCM doorgestuurd naar GPS (%d bytes)", w);
                }
            }
            close(sock);
            sleep(5);
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
        if (!fields[9].empty()) {
            alt = std::stod(fields[9]);
        }

        int fix_quality = 0;
        if (!fields[6].empty()) {
            fix_quality = std::stoi(fields[6]);
        }

        std::string mode = (fix_quality == 4) ? "RTK Fixed" :
                           (fix_quality == 5) ? "RTK Float" :
                           (fix_quality == 2) ? "DGPS" :
                           (fix_quality == 1) ? "GPS" : "No Fix";

        // ✅ Alleen loggen als status verandert
        if (fix_quality != last_fix_quality_) {
            RCLCPP_INFO(this->get_logger(), "GPS status veranderd → %s", mode.c_str());
            last_fix_quality_ = fix_quality;
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

    // Config
    std::string device_;
    int baudrate_;
    int fd_;
    std::string buffer_;
    std::thread reader_thread_, rtcm_thread_;
    std::string rtcm_host_;
    int rtcm_port_;
    int last_fix_quality_;   // <-- toegevoegd

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_raw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNode>());
    rclcpp::shutdown();
    return 0;
}

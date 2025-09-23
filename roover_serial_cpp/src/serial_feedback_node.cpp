#include "rclcpp/rclcpp.hpp"
#include "roover_serial_cpp/msg/roover_feedback.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <cerrno>

//using std::placeholders::_1;
using roover_serial_cpp::msg::RooverFeedback;

class SerialFeedbackNode : public rclcpp::Node
{
public:
    SerialFeedbackNode()
    : Node("serial_feedback_node")
    {
        // Declareer ROS2 parameters
        this->declare_parameter<std::string>("device", "/dev/ttyAMA0");
        this->declare_parameter<int>("baudrate", 115200);

        // Lees parameters in
        this->get_parameter("device", dev_);
        this->get_parameter("baudrate", baud_);

        // Seriële poort openen
        //fd_ = open(dev_.c_str(), O_RDWR | O_NOCTTY);
        fd_ = open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Kan seriële poort niet openen: %s",
                dev_.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Geopend serial port: %s", dev_.c_str());

        // Zet terug naar blocking mode
        fcntl(fd_, F_SETFL, 0);
        
        // Configureer baudrate en raw mode
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(),
                   "tcgetattr faalt: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        cfmakeraw(&tty);
        cfsetispeed(&tty, baud_);
        cfsetospeed(&tty, baud_);
        tty.c_cflag |= (CLOCAL | CREAD);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(),
                   "tcsetattr faalt: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Kick-byte sturen om GD32 te triggeren
        const uint8_t kick = 0x00;
        write(fd_, &kick, 1);
        RCLCPP_INFO(this->get_logger(), "Kick-byte gestuurd om feedback te starten");

        // Maak ROS2 publisher
        publisher_ = this->create_publisher<RooverFeedback>("roover_feedback", rclcpp::QoS(10));

        // Start lees-loop via timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&SerialFeedbackNode::on_timer, this)
        );
    }

    ~SerialFeedbackNode() override
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    void on_timer()
    {
        uint8_t buf[64];
        int n = read(fd_, buf, sizeof(buf));
        if (n <= 0) return;
        accum_.insert(accum_.end(), buf, buf + n);

        // Blijf zoeken zolang er genoeg bytes in de buffer zitten
        while (accum_.size() >= 2 /* voor start-id */) {
            // 1) Zoek het start-woord (little endian voorbeeld: 0xCDAB)
            bool found = false;
            for (size_t i = 0; i + 1 < accum_.size(); ++i) {
                uint16_t candidate = accum_[i] | (accum_[i+1] << 8);
                if (candidate == 0xABCD || candidate == 0xCDAB) {
                    // accepteer beide endian-varianten
                    // Verwijder alle bytes vóór het start-woord
                    accum_.erase(accum_.begin(), accum_.begin() + i);
                    found = true;
                    break;
                }
            }
            if (!found) {
            // Geen enkel start-woord meer in buffer: wist alles
            accum_.clear();
            break;
            }

            // 2) Zodra start-woord op positie 0, wacht tot we hele frame hebben
            if (accum_.size() < sizeof(FeedbackFrame)) {
            // nog niet compleet, wacht op volgende read()
            break;
            }

            while (accum_.size() >= sizeof(FeedbackFrame)) {
            // 1) Kopieer de bytes naar je frame‐struct
                FeedbackFrame fb;
                std::memcpy(&fb, accum_.data(), sizeof(fb));

                // 2) Bereken de checksum exact zoals in de firmware
                uint16_t calc = 
                    fb.start 
                    ^ fb.cmd1 
                    ^ fb.cmd2 
                    ^ fb.speed_l 
                    ^ fb.speed_r 
                    ^ fb.wheel_l_cnt 
                    ^ fb.wheel_r_cnt 
                    ^ fb.battery_voltage 
                    ^ fb.board_temp 
                    ^ fb.cmd_led
                    ;

                // 3) Vergelijk met wat de firmware meegestuurd heeft
                if (calc == fb.checksum) {
                    // ✅ geldig frame: publiceer
                    // 4) Publiceer je ROS‐bericht
                    RooverFeedback msg;
                    msg.start	        = fb.start;
                    msg.cmd1            = fb.cmd1;
                    msg.cmd2            = fb.cmd2;
                    msg.speed_l         = fb.speed_l;
                    msg.speed_r         = fb.speed_r;
                    msg.wheel_l_cnt     = fb.wheel_l_cnt;
                    msg.wheel_r_cnt     = fb.wheel_r_cnt;
                    msg.battery_voltage = fb.battery_voltage;
                    msg.board_temp      = fb.board_temp;
                    msg.cmd_led         = fb.cmd_led;
                    msg.checksum        = fb.checksum;
                    publisher_->publish(msg);

                    // Verwijder wél dit complete, gevalideerde frame uit de buffer
                accum_.erase(accum_.begin(),
                        accum_.begin() + sizeof(fb));

                } else {
                    RCLCPP_WARN(this->get_logger(), "Ongeldige checksum: %04X != %04X",
                        calc, fb.checksum);
                    accum_.erase(accum_.begin());
                }
            }
        }
    }
            

    // Struct overeenkomend met de firmware-packet layout
    struct FeedbackFrame {
        uint16_t start;
        int16_t  cmd1;
        int16_t  cmd2;
        int16_t  speed_r;
        int16_t  speed_l;
        int16_t  wheel_r_cnt;
        int16_t  wheel_l_cnt;
        int16_t  battery_voltage;
        int16_t  board_temp;
        uint16_t cmd_led;
        uint16_t checksum;
    };

    std::string dev_;
    int         baud_;
    int         fd_{-1};

    rclcpp::Publisher<RooverFeedback>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> accum_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialFeedbackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

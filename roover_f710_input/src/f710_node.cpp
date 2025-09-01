#include "rclcpp/rclcpp.hpp"
#include "roover_f710_input/msg/f710_input.hpp"  // juiste include
#include <SDL2/SDL.h>

using F710Input = roover_f710_input::msg::F710Input;  // namespace van jóuw package

class F710Node : public rclcpp::Node
{
public:
  F710Node()
  : Node("f710_node")
  {
    // publisher met QoS-construct (niet raw int)
    publisher_ = this->create_publisher<F710Input>(
      "f710_input", rclcpp::QoS(10)
    );

    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
      RCLCPP_FATAL(this->get_logger(), "SDL kon niet initialiseren: %s", SDL_GetError());
      rclcpp::shutdown();
      return;
    }

    int n = SDL_NumJoysticks();
    RCLCPP_INFO(this->get_logger(), "SDL detecteerde %d joysticks", n);
    for (int i = 0; i < n; ++i) {
      RCLCPP_INFO(this->get_logger(), "  %d: %s", i, SDL_JoystickNameForIndex(i));
    }

    controller_ = SDL_GameControllerOpen(0);
    if (!controller_) {
      RCLCPP_FATAL(this->get_logger(), "Geen F710 controller gevonden!");
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&F710Node::read_joystick, this)
    );
  }

  ~F710Node() override
  {
    if (controller_) {
      SDL_GameControllerClose(controller_);
    }
    SDL_Quit();
  }

private:
  void read_joystick()
  {
    SDL_GameControllerUpdate();
    F710Input msg;
    msg.header.stamp = now();
    msg.header.frame_id = "f710";

    msg.left_x  = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTX);
    msg.left_y  = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTY);
    msg.right_x = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_RIGHTX);
    msg.right_y = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_RIGHTY);

    msg.button_a     = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_A);
    msg.button_b     = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_B);
    msg.button_x     = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_X);
    msg.button_y     = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_Y);
    msg.button_lb    = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
    msg.button_rb    = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
    msg.button_back  = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_BACK);
    msg.button_start = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_START);
    msg.button_home  = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_GUIDE);

    msg.dpad_up    = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_UP);
    msg.dpad_down  = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
    msg.dpad_left  = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
    msg.dpad_right = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);

    publisher_->publish(msg);
  }

  SDL_GameController * controller_{nullptr};
  rclcpp::Publisher<F710Input>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<F710Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

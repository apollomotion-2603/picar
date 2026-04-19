#include "keyboard_controller.hpp"

#include <unistd.h>   // read(), STDIN_FILENO
#include <fcntl.h>    // fcntl(), F_SETFL, O_NONBLOCK


// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
KeyboardController::KeyboardController(double timer_period)
: Node{"keyboard_controller"},
  velocity_{0.0},
  steering_{0.0},
  running_{true}
{
  // ── Parameters ─────────────────────────────────────────────────────────────
  declare_parameter<double>("max_steering_angle", 0.5);
  declare_parameter<double>("max_velocity",       2.0);
  declare_parameter<double>("step_velocity",      0.1);
  declare_parameter<double>("step_steering",      0.05);

  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity",       max_velocity_);
  get_parameter("step_velocity",      step_velocity_);
  get_parameter("step_steering",      step_steering_);

  // ── Publishers ─────────────────────────────────────────────────────────────
  velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/velocity",       1);
  steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 1);

  // ── Timer ──────────────────────────────────────────────────────────────────
  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&KeyboardController::timer_callback, this));

  // ── Raw terminal + keyboard thread ─────────────────────────────────────────
  enable_raw_mode();
  key_thread_ = std::thread(&KeyboardController::key_loop, this);

  RCLCPP_INFO(get_logger(),
    "\n================================================\n"
    "  Keyboard Controller  (raw-terminal mode)\n"
    "  ↑ / ↓        velocity  +/-  (step=%.2f m/s)\n"
    "  ← / →        steering  L/R  (step=%.2f rad)\n"
    "  Space        velocity = 0\n"
    "  r            reset all\n"
    "  q            quit\n"
    "================================================",
    step_velocity_, step_steering_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Destructor — restore terminal on exit
// ─────────────────────────────────────────────────────────────────────────────
KeyboardController::~KeyboardController()
{
  running_ = false;
  if (key_thread_.joinable()) key_thread_.join();
  disable_raw_mode();
}

// ─────────────────────────────────────────────────────────────────────────────
// Terminal helpers
// ─────────────────────────────────────────────────────────────────────────────
void KeyboardController::enable_raw_mode()
{
  tcgetattr(STDIN_FILENO, &original_termios_);  // Save original terminal settings

  struct termios raw = original_termios_;       // Start with original settings
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO); // no line-buffer, no echo
  raw.c_cc[VMIN]  = 0;  // non-blocking read
  raw.c_cc[VTIME] = 0;  // no read timeout

  tcsetattr(STDIN_FILENO, TCSANOW, &raw); // Apply raw settings
}

void KeyboardController::disable_raw_mode()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_); // Restore original settings
}

// ─────────────────────────────────────────────────────────────────────────────
// key_loop — runs in a dedicated thread, polls stdin at ~100 Hz
//
// Arrow key escape sequence:
//   ESC (0x1B)  →  '['  →  'A' up / 'B' down / 'C' right / 'D' left
// ─────────────────────────────────────────────────────────────────────────────
void KeyboardController::key_loop()
{
  while (running_) {
    unsigned char buf[3] = {0, 0, 0};
    ssize_t n = read(STDIN_FILENO, buf, 1);

    if (n <= 0) {
      // Nothing to read — sleep 10 ms to avoid busy-spin
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    double d = velocity_.load();
    double s = steering_.load();

    if (buf[0] == 0x1B) {
      // Possibly an escape sequence — read 2 more bytes
      if (read(STDIN_FILENO, buf + 1, 1) == 1 && buf[1] == '[') {
        if (read(STDIN_FILENO, buf + 2, 1) == 1) {
          switch (buf[2]) {
            case 'A':  // ↑  velocity up
              d = std::max(-max_velocity_, std::min(max_velocity_, d + step_velocity_));
              RCLCPP_INFO(get_logger(), "↑  velocity = %.2f m/s", d);
              break;
            case 'B':  // ↓  velocity down
              d = std::max(-max_velocity_, std::min(max_velocity_, d - step_velocity_));
              RCLCPP_INFO(get_logger(), "↓  velocity = %.2f m/s", d);
              break;
            case 'C':  // →  steer right (negative)
              s = std::max(-max_steering_angle_, std::min(max_steering_angle_, s - step_steering_));
              RCLCPP_INFO(get_logger(), "→  steering = %.3f rad", s);
              break;
            case 'D':  // ←  steer left (positive)
              s = std::max(-max_steering_angle_, std::min(max_steering_angle_, s + step_steering_));
              RCLCPP_INFO(get_logger(), "←  steering = %.3f rad", s);
              break;
            default:
              break;
          }
        }
      }
    } else {
      switch (buf[0]) {
        case ' ':  // Space — kill velocity
          d = 0.0;
          RCLCPP_INFO(get_logger(), "Space  velocity = 0");
          break;
        case 'r':  // r — full reset
          d = 0.0;
          s = 0.0;
          RCLCPP_INFO(get_logger(), "r      reset: throttle=0, steering=0");
          break;
        case 'q':  // q — quit
          RCLCPP_INFO(get_logger(), "q      shutting down...");
          rclcpp::shutdown();
          return;
        default:
          break;
      }
    }

    velocity_.store(d);
    steering_.store(s);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// timer_callback — publishes at fixed rate (default 20 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void KeyboardController::timer_callback()
{
  std_msgs::msg::Float64 v_msg, s_msg;
  v_msg.data = velocity_.load();
  s_msg.data = steering_.load();

  velocity_pub_->publish(v_msg);
  steering_pub_->publish(s_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardController>());
  rclcpp::shutdown();
  return 0;
}

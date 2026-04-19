#pragma once

#include <atomic>
#include <thread>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// KeyboardController
//
// Đọc phím mũi tên từ terminal (raw mode, non-blocking) trong một thread riêng.
// Publish:
//   /velocity        (Float64)  v [m/s]  — trực tiếp tới vehicle_controller
//   /steering_angle  (Float64)  δ ∈ [-max_steering_angle, +max_steering_angle]
//
// Phím điều khiển:
//   ↑  / ↓   : tăng / giảm velocity (bước step_velocity_)
//   ← / →   : rẽ trái / phải (bước step_steering_)
//   Space    : velocity = 0  (dừng xe)
//   r        : reset toàn bộ (velocity = 0, steering = 0)
//   q / Ctrl-C : thoát node
// ─────────────────────────────────────────────────────────────────────────────

class KeyboardController : public rclcpp::Node
{
public:
  explicit KeyboardController(double timer_period = 5e-2);
  ~KeyboardController();

private:
  // ── Publishers & timer ────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
  rclcpp::TimerBase::SharedPtr                         timer_;

  // ── State (modified by keyboard thread, read by timer thread) ─────────
  std::atomic<double> velocity_; // v  [m/s]
  std::atomic<double> steering_; // δ  ∈ [-max_steering_angle_, +max_steering_angle_]

  // ── Parameters ────────────────────────────────────────────────
  double max_steering_angle_; // [rad]
  double max_velocity_;       // [m/s]
  double step_velocity_;      // v increment per key press [m/s]
  double step_steering_;      // δ increment per key press [rad]

  // ── Terminal raw mode ─────────────────────────────────────────────────────
  struct termios original_termios_;
  void enable_raw_mode();
  void disable_raw_mode();

  // ── Keyboard reader thread ─────────────────────────────────────────────────
  std::thread       key_thread_;
  std::atomic<bool> running_;
  void key_loop();

  // ── Timer callback ────────────────────────────────────────────────────────
  void timer_callback();
};

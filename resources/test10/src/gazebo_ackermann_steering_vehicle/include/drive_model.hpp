#pragma once

#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// DriveModel
//
// Mô hình động lực học dọc trục xe:
//
//   F_xᵈ = (cm1 - cm2·v)·D - cr2·v² - cr0·tanh(cr3·v)
//
// Subscribe:
//   /throttle                    (Float64)   D  — duty cycle từ NMPC/keyboard
//   /ground_truth/odom/filtered  (Odometry)  v  — filtered velocity (body frame)
//
// Publish:
//   /velocity    (Float64)  v [m/s]   — relay tới vehicle_controller
//   /drive_force (Float64)  F_xᵈ [N] — debug only
//
// Parameters:
//   cm1          motor thrust constant          [default: 0.287]
//   cm2          back-EMF speed-loss coeff      [default: 0.054]
//   cr2          aerodynamic drag (quadratic)   [default: 0.0  ]
//   cr0          Coulomb / rolling friction     [default: 0.051]
//   cr3          tanh shaping factor            [default: 50.0 ]
//   vehicle_mass mass of vehicle m [kg]         [default: 1.5  ]
//   max_velocity maximum allowed |v| [m/s]      [default: 2.0  ]
// ─────────────────────────────────────────────────────────────────────────────

class DriveModel : public rclcpp::Node
{
public:
  explicit DriveModel(double timer_period = 5e-2);

private:
  // ── Compute F_xᵈ ─────────────────────────────────────────────────────────
  double compute_drive_force(double D, double v) const;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void timer_callback();

  // ── Publishers & Subscribers ──────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  throttle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     force_pub_;   // debug
  rclcpp::TimerBase::SharedPtr                             timer_;

  // ── State ─────────────────────────────────────────────────────────────────
  double D_;            // current duty cycle input
  double v_;            // latest filtered velocity from odom [m/s]
  double timer_period_; // integration time step [s]

  // ── Model parameters ──────────────────────────────────────────────────────
  double cm1_;
  double cm2_;
  double cr2_;
  double cr0_;
  double cr3_;
  double mass_;
  double max_velocity_;
};

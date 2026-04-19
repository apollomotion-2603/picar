#include "drive_model.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
DriveModel::DriveModel(double timer_period)
: Node{"drive_model"},
  D_{0.0},
  v_{0.0},
  timer_period_{timer_period}
{
  // ── Parameters ─────────────────────────────────────────────────────────────
  declare_parameter<double>("cm1",          0.287);
  declare_parameter<double>("cm2",          0.054);
  declare_parameter<double>("cr2",          0.0);
  declare_parameter<double>("cr0",          0.051);
  declare_parameter<double>("cr3",         50.0);
  declare_parameter<double>("vehicle_mass", 1.5);
  declare_parameter<double>("max_velocity", 2.0);

  get_parameter("cm1",          cm1_);
  get_parameter("cm2",          cm2_);
  get_parameter("cr2",          cr2_);
  get_parameter("cr0",          cr0_);
  get_parameter("cr3",          cr3_);
  get_parameter("vehicle_mass", mass_);
  get_parameter("max_velocity", max_velocity_);

  // ── Subscribers ────────────────────────────────────────────────────────────
  throttle_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/throttle", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) { D_ = msg->data; });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/ground_truth/odom/filtered", 1,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      v_ = msg->twist.twist.linear.x;
    });

  // ── Publishers ─────────────────────────────────────────────────────────────
  velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/velocity",    1);
  force_pub_    = create_publisher<std_msgs::msg::Float64>("/drive_force", 1);

  // ── Integration timer ──────────────────────────────────────────────────────
  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&DriveModel::timer_callback, this));

  RCLCPP_INFO(get_logger(),
    "DriveModel ready  [cm1=%.3f cm2=%.3f cr0=%.3f cr2=%.3f cr3=%.1f m=%.2f kg]",
    cm1_, cm2_, cr0_, cr2_, cr3_, mass_);
  RCLCPP_INFO(get_logger(), "Velocity source: /ground_truth/odom/filtered");
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_drive_force
//
//   F_xᵈ = (cm1 - cm2·v)·D  −  cr2·v²  −  cr0·tanh(cr3·v)
//            └──── motor ────┘   └─ aero─┘   └── friction ──┘
// ─────────────────────────────────────────────────────────────────────────────
double DriveModel::compute_drive_force(double D, double v) const
{
  const double motor    = (cm1_ - cm2_ * v) * D;
  const double aero     = cr2_ * v * v;
  const double friction = cr0_ * std::tanh(cr3_ * v);
  return motor - aero - friction;
}

// ─────────────────────────────────────────────────────────────────────────────
// timer_callback — integrate F_xᵈ to produce velocity command
//
//   v̇ = (F_xᵈ / m) · cos(β)   (Eq. 10d, paper Kloeser et al.)
//
// cos(β) ≈ 1 for small side-slip angle β.
// Uses Euler integration:  v_cmd = v_actual + (F_xᵈ / m) · Δt
// ─────────────────────────────────────────────────────────────────────────────
void DriveModel::timer_callback()
{
  // ── 1. Convert D from [-1,1] to [-255,255] (8-bit duty cycle) ─────────────
  double D_scaled = D_ * 255.0;
  // Clamp to [-255, 255] just in case
  D_scaled = std::clamp(D_scaled, -255.0, 255.0);
  // Compute drive force using scaled duty cycle
  const double Fd = compute_drive_force(D_scaled, v_);

  // ── 2. Integrate: v_cmd = v_actual + a·Δt  (Eq. 10d) ──────────────────────
  const double a     = Fd / mass_;
  double       v_cmd = v_ + a * timer_period_;
  v_cmd = std::clamp(v_cmd, -max_velocity_, max_velocity_);

  std_msgs::msg::Float64 vel_msg;
  vel_msg.data = v_cmd;
  velocity_pub_->publish(vel_msg);

  // ── 3. Publish /drive_force (debug) ───────────────────────────────────────
  std_msgs::msg::Float64 force_msg;
  force_msg.data = Fd;
  force_pub_->publish(force_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveModel>());
  rclcpp::shutdown();
  return 0;
}

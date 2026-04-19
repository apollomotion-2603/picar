#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// OdomFilterNode
//   Subscribes to /ground_truth/odom, applies a deadband to all twist fields,
//   and republishes the filtered message on /ground_truth/odom/filtered.
//
//   Parameters:
//     linear_deadband  [m/s]    values with |v| < threshold → 0  (default 0.001)
//     angular_deadband [rad/s]  values with |w| < threshold → 0  (default 0.001)
// ─────────────────────────────────────────────────────────────────────────────
class OdomFilterNode : public rclcpp::Node
{
public:
  OdomFilterNode()
  : Node("odom_filter")
  {
    declare_parameter<double>("linear_deadband",  0.001);
    declare_parameter<double>("angular_deadband", 0.001);

    get_parameter("linear_deadband",  linear_db_);
    get_parameter("angular_deadband", angular_db_);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 10,
      std::bind(&OdomFilterNode::odom_callback, this, std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/ground_truth/odom/filtered", 10);

    RCLCPP_INFO(get_logger(),
      "odom_filter ready  [linear_deadband=%.4f m/s  angular_deadband=%.4f rad/s]",
      linear_db_, angular_db_);
  }

private:
  static double deadband(double value, double threshold)
  {
    return (std::abs(value) < threshold) ? 0.0 : value;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;

    // Convert global velocity to body-frame velocity using yaw
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    
    double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double v_x = msg->twist.twist.linear.x;
    double v_y = msg->twist.twist.linear.y;
    
    double v_long = v_x * std::cos(yaw) + v_y * std::sin(yaw);
    double v_lat  = -v_x * std::sin(yaw) + v_y * std::cos(yaw);

    out.twist.twist.linear.x  = deadband(v_long,  linear_db_);
    out.twist.twist.linear.y  = deadband(v_lat,   linear_db_);
    out.twist.twist.linear.z  = deadband(msg->twist.twist.linear.z,  linear_db_);
    out.twist.twist.angular.x = deadband(msg->twist.twist.angular.x, angular_db_);
    out.twist.twist.angular.y = deadband(msg->twist.twist.angular.y, angular_db_);
    out.twist.twist.angular.z = deadband(msg->twist.twist.angular.z, angular_db_);

    pub_->publish(out);
  }

  double linear_db_{0.001};
  double angular_db_{0.001};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFilterNode>());
  rclcpp::shutdown();
  return 0;
}

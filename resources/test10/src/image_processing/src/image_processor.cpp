#include <sensor_msgs/msg/camera_info.hpp>
#include "image_declarations.h"

// ── Constructor (public) ──────────────────────────────────────────────────────
ImageProcessor::ImageProcessor() : Node("image_processor")
{
    this->declare_parameter("scan_top_ratio",   0.45);  // bắt đầu quét từ 45% chiều cao ảnh (bỏ phần trời)
    this->declare_parameter("scan_bot_ratio",   0.98);  // dừng quét tại 98% chiều cao ảnh (bỏ gầm xe)
    this->declare_parameter("black_threshold",  100);   // pixel < 100 → coi là vạch đen
    this->declare_parameter("band_height",      6);     // chiều dày mỗi scan band [px]
    scan_top_ratio_  = this->get_parameter("scan_top_ratio").as_double();
    scan_bot_ratio_  = this->get_parameter("scan_bot_ratio").as_double();
    black_threshold_ = this->get_parameter("black_threshold").as_int();
    band_height_     = this->get_parameter("band_height").as_int();

    // ── Subscribers / Publishers ─────────────────────────────────────
    // Tạo subscriber lắng nghe topic ảnh thô từ camera; mỗi khi có frame mới,
    // ROS 2 sẽ gọi image_callback để xử lý. Queue size=10 cho phép giữ tối đa
    // 10 message trong hàng đợi nếu callback xử lý không kịp tốc độ publish.
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/vehicle_front_camera/image_raw", 10,
      std::bind(&ImageProcessor::image_callback, this, _1));

      // Subscribe to camera info
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/vehicle_front_camera/camera_info", 10,
      std::bind(&ImageProcessor::camera_info_callback, this, std::placeholders::_1));  

    pub_ref_path_      = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/reference_path", 10);
    pub_cte_           = this->create_publisher<std_msgs::msg::Float64>("/line/cte", 10);
    pub_heading_err_ = this->create_publisher<std_msgs::msg::Float64>("/line/heading_error", 10);
    pub_poly_coeffs_   = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/poly_coeffs", 10);
    pub_debug_         = this->create_publisher<sensor_msgs::msg::Image>("/line/debug_image", 10);
    pub_fitting_image_ = this->create_publisher<sensor_msgs::msg::Image>("/line/fitting_image", 10);
    pub_kappa_         = this->create_publisher<std_msgs::msg::Float64>("/line/kappa", 10);

    // Thêm publisher cho heading_angle, s_metric và kappa_s
    pub_heading_angle_  = this->create_publisher<std_msgs::msg::Float64>("/line/heading_angle", 10);
    pub_s_             = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/s_metric", 10);
    pub_kappa_s_       = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/kappa_s", 10);

    // RCLCPP_INFO(this->get_logger(),
    //   "ImageProcessor started | bands=%d | scan=[%.2f, %.2f] | threshold=%d",
    //   N, scan_top_ratio_, scan_bot_ratio_, black_threshold_);
}

// ── ROS 2 entry point ────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}

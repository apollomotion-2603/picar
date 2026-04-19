#include <sensor_msgs/msg/camera_info.hpp>
#include "image_declarations.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

#include <algorithm>

// ====== Polyfit cho metric và curvature ======
#include <Eigen/Dense>

// Fit đa thức bậc n: y = poly[0] + poly[1]*x + poly[2]*x^2 + ...
std::vector<double> poly_fit_metric(const std::vector<double>& x, const std::vector<double>& y, int order) {
  int N = x.size();
  Eigen::MatrixXd A(N, order);
  Eigen::VectorXd Y(N);
  for (int i = 0; i < N; ++i) {
    double v = 1.0;
    for (int j = 0; j < order; ++j) {
      A(i, j) = v;
      v *= x[i];
    }
    Y(i) = y[i];
  }
  // Least squares fit
  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(Y);
  std::vector<double> poly(order);
  for (int i = 0; i < order; ++i) poly[i] = coeffs(i);
  return poly;
}

// Tính đạo hàm bậc 1, 2 của đa thức tại x
void poly_derivatives(const std::vector<double>& poly, double x, double& d1, double& d2) {
  d1 = 0.0; d2 = 0.0;
  int n = poly.size();
  for (int i = 1; i < n; ++i) {
    d1 += i * poly[i] * std::pow(x, i-1);
  }
  for (int i = 2; i < n; ++i) {
    d2 += i * (i-1) * poly[i] * std::pow(x, i-2);
  }
}

// Tính curvature tại từng điểm (fit X = f(Y))
std::vector<double> compute_curvature(const std::vector<double>& Y, const std::vector<double>& X_poly) {
  int N = Y.size();
  std::vector<double> kappa(N, 0.0);
  for (int i = 0; i < N; ++i) {
    double d1, d2;
    poly_derivatives(X_poly, Y[i], d1, d2);
    kappa[i] = std::abs(d2) / std::pow(1 + d1*d1, 1.5);
  }
  return kappa;
}

// ====== Homography & Pixel-to-Metric Utilities ======
// (Bạn cần truyền các tham số này từ config hoặc camera_info)
namespace {
// Example: hardcode, bạn nên lấy từ config/params.yaml
constexpr double fx = 600.0; // focal length x (pixel)
constexpr double fy = 600.0; // focal length y (pixel)
constexpr double cx = 320.0; // principal point x (pixel)
constexpr double cy = 240.0; // principal point y (pixel)
// constexpr double camera_height = 0.165; // [m]
constexpr double camera_height = 0.165; // [m] (đã bao gồm body height + wheel radius)
// camera_height_ground=camera_height+body_height/2+wheel_radius
// constexpr double camera_pitch_deg = 20.0; // [deg]
constexpr double camera_pitch_rad = 0.2; // [rad]

cv::Mat build_homography_matrix(const cv::Mat& K)
{
  // Rotation: pitch around X axis
  // double pitch = camera_pitch_deg * M_PI / 180.0;
  double pitch = camera_pitch_rad;
  cv::Mat R = (cv::Mat_<double>(3,3) <<
    1,         0,          0,
    0,  std::cos(pitch), -std::sin(pitch),
    0,  std::sin(pitch),  std::cos(pitch));
  // Translation: camera height above ground
  cv::Mat t = (cv::Mat_<double>(3,1) << 0, 0, camera_height);
  // Normal of ground plane: [0, 0, 1], d = 0
  cv::Mat n = (cv::Mat_<double>(3,1) << 0, 0, 1);
  // Homography: H = K * [R - t*n^T/d] * K^{-1}, but for ground at Z=0:
  // H = K * (R - t*n^T/camera_height) * K.inv()
  cv::Mat Rt = R - t * n.t() / camera_height;
  cv::Mat H = K * Rt * K.inv();
  return H;
}

// Chuyển pixel (u,v) sang (X,Y) trên mặt phẳng đất
cv::Point2d pixel_to_ground(double u, double v, const cv::Mat& H)
{
  cv::Mat pt = (cv::Mat_<double>(3,1) << u, v, 1.0);
  cv::Mat ground = H.inv() * pt;
  double X = ground.at<double>(0,0) / ground.at<double>(2,0);
  double Y = ground.at<double>(1,0) / ground.at<double>(2,0);
  return cv::Point2d(X, Y);
}
}

// Tính arc length s tại các điểm y, với đa thức x = c0 + c1*y + c2*y^2
std::vector<double> compute_arc_length_s(const std::vector<double>& y, const std::vector<double>& poly_coeffs) {
  int N = y.size();
  std::vector<double> s(N, 0.0);
  if (poly_coeffs.size() < 3) return s;
  double c1 = poly_coeffs[1];
  double c2 = poly_coeffs[2];
  for (int i = 1; i < N; ++i) {
    double y0 = y[i-1];
    double y1 = y[i];
    double dy = y1 - y0;
    double dx1 = c1 + 2 * c2 * y0;
    double dx2 = c1 + 2 * c2 * y1;
    double ds = 0.5 * (std::sqrt(1 + dx1*dx1) + std::sqrt(1 + dx2*dx2)) * std::abs(dy);
    s[i] = s[i-1] + ds;
  }
  return s;
}

double ImageProcessor::scan_row(const unsigned char* data, int row, int width, int step, int channels)
{
    int row_off = row * step;
    int left = -1, right = -1;
    for (int x = 0; x < width; x++) {
        int brightness = data[row_off + x * channels];
        if (brightness < black_threshold_) {
            if (left == -1) left = x;
            right = x;
        }
    }
    if (left != -1 && right != -1) {
        return (left + right) / 2.0;
    }
    return -1.0;
} 

void ImageProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!has_camera_info_) {
    RCLCPP_WARN(this->get_logger(), "[IMG] Chưa nhận được CameraInfo, bỏ qua frame!");
    return;
  }
    if (!check_valid_image(msg)) return; // Chỉ hỗ trợ ảnh RGB8 hoặc BGR8 (1 byte mỗi kênh, 3 kênh tổng cộng)

    std::vector<double> lateral_offset(N, 0.0);
    const int width    = static_cast<int>(msg->width);
    const int height   = static_cast<int>(msg->height);
    const int step     = static_cast<int>(msg->step); // stride amount of bytes per row
    const unsigned char* data = msg->data.data(); // truy cập đến từng pixel ảnh

    const int channels = step / width;
    const double image_center = width / 2.0;
    int scan_top = static_cast<int>(height * scan_top_ratio_);
    int scan_bot = static_cast<int>(height * scan_bot_ratio_);
    int span     = scan_bot - scan_top;

    std::vector<int> scan_rows(N);
    bool any_valid = compute_offsets(data, width, height, step, channels, 
        image_center, scan_bot, span, lateral_offset, scan_rows);

    if (!any_valid) {
      RCLCPP_WARN(this->get_logger(), "[IMG] Không thấy vạch đen ở tất cả %d scan lines!", N);
      return;
    }

    // ── Chuyển lateral_offset (pixel) sang metric bằng homography và log thử ──
    cv::Mat H = build_homography_matrix(K_);
    std::vector<double> lateral_offset_metric(N);
    for (int i = 0; i < N; ++i) {
        int row = scan_rows[i];
        double u = image_center + lateral_offset[i]; // vị trí pixel thực tế
        double u_center = image_center; // tâm ảnh
        // Chuyển sang metric
        cv::Point2d pt = pixel_to_ground(u, row, H);
        cv::Point2d pt_center = pixel_to_ground(u_center, row, H);
        lateral_offset_metric[i] = pt.x - pt_center.x;
    }
    // Log kết quả
    std::ostringstream oss_lateral;
    oss_lateral << "[lateral_offset_metric (m)]: ";
    for (int i = 0; i < N; ++i) {
        oss_lateral << lateral_offset_metric[i];
        if (i < N - 1) oss_lateral << ", ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss_lateral.str().c_str());
    // Log giá trị lateral offset (m) gần xe nhất (scanline cuối)
    RCLCPP_INFO(this->get_logger(), "[lateral_offset_metric_nearest (m)]: %f", lateral_offset_metric[N-1]);

    // Góc nghiêng tổng thể của đường so với trục ngang ảnh có thể ước tính từ độ dốc của các điểm centroid.
    // double delta_lateral = lateral_offset[N-1] - lateral_offset[0];
    // double pixel_dist    = static_cast<double>(scan_rows[0] - scan_rows[N-1]);
    // double heading_error = std::atan2(delta_lateral, pixel_dist);  // [rad]

    // ── Polynomial fit bậc 2 ─────────────────────────────────────────
    // centroid_x[i] = toạ độ x tuyệt đối (pixel) của tâm làn tại scan_rows[i]
    std::vector<double> centroid_x(N);
    for (int i = 0; i < N; i++)
        centroid_x[i] = lateral_offset[i] + image_center;
    std::vector<double> poly_coeffs = poly_fit(scan_rows, centroid_x, 3);


    // ...existing code...
    // ── Chuyển centroid (pixel) sang metric (m) bằng homography ──
    std::vector<cv::Point2d> ground_points(N);
    for (int i = 0; i < N; ++i) {
      ground_points[i] = pixel_to_ground(centroid_x[i], scan_rows[i], H);
    }

  // ...existing code...
    std::vector<double> ground_Y(N), ground_X(N);
    for (int i = 0; i < N; ++i) {
        ground_X[i] = ground_points[i].x;
        ground_Y[i] = ground_points[i].y;
    }
  // ── Publish lateral_offset_metric (m, so với tâm ảnh) ────────────────
  std_msgs::msg::Float64MultiArray ref_msg;
  ref_msg.data = lateral_offset_metric;
  pub_ref_path_->publish(ref_msg);
  std_msgs::msg::Float64 cte_msg;
  cte_msg.data = lateral_offset_metric[N - 1];
  pub_cte_->publish(cte_msg);

    // poly_coeffs: [c0, c1, c2]  →  x = c0 + c1*y + c2*y²
    std_msgs::msg::Float64MultiArray poly_msg;
    poly_msg.data = poly_coeffs;
    pub_poly_coeffs_->publish(poly_msg);


  // Fit đa thức X = f(Y) trên metric
  std::vector<double> X_poly = poly_fit_metric(ground_Y, ground_X, 3); // bậc 2

  // === Tính heading angle từ đa thức metric ===
  double Y0 = ground_Y[N-1]; // điểm gần xe nhất (tùy hướng scan)
  double dX_dY = X_poly[1] + 2 * X_poly[2] * Y0;

  double heading_angle = std::atan(dX_dY); // [rad]
  RCLCPP_INFO(this->get_logger(), "[heading_angle (rad)]: %f", heading_angle);

  // Publish heading_angle as a ROS topic
  std_msgs::msg::Float64 heading_angle_msg;
  heading_angle_msg.data = heading_angle;
  if (pub_heading_angle_) pub_heading_angle_->publish(heading_angle_msg);

  // Tính arc length analytic trên metric (m)
  std::vector<double> s_metric = compute_arc_length_s(ground_Y, X_poly);
  // Tính curvature tại từng Y
  std::vector<double> kappa = compute_curvature(ground_Y, X_poly);
  // Log analytic arc length và curvature
  std::ostringstream oss;
  oss << "[arc_length analytic (m)]: ";
  for (size_t i = 0; i < s_metric.size(); ++i) {
    oss << s_metric[i];
    if (i < s_metric.size() - 1) oss << ", ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  std::ostringstream oss2;
  oss2 << "[curvature kappa]: ";
  for (size_t i = 0; i < kappa.size(); ++i) {
    oss2 << kappa[i];
    if (i < kappa.size() - 1) oss2 << ", ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", oss2.str().c_str());

  // ===== Curvature theo arc length s (kappa_s) =====
  // Fit lại đa thức X = f(s), Y = f(s)
  std::vector<double> X_poly_s = poly_fit_metric(s_metric, ground_X, 3);
  std::vector<double> Y_poly_s = poly_fit_metric(s_metric, ground_Y, 3);
  // Tính curvature theo s
  std::vector<double> kappa_s(s_metric.size());
  for (size_t i = 0; i < s_metric.size(); ++i) {
      double s = s_metric[i];
      double dx1, dx2, dy1, dy2;
      poly_derivatives(X_poly_s, s, dx1, dx2);
      poly_derivatives(Y_poly_s, s, dy1, dy2);
      double num = std::abs(dx1 * dy2 - dy1 * dx2);
      double denom = std::pow(dx1*dx1 + dy1*dy1, 1.5);
      kappa_s[i] = denom > 1e-8 ? num / denom : 0.0;
  }
  // Log curvature theo arc length
  std::ostringstream oss3;
  oss3 << "[curvature kappa_s (arc length)]: ";
  for (size_t i = 0; i < kappa_s.size(); ++i) {
    oss3 << kappa_s[i];
    if (i < kappa_s.size() - 1) oss3 << ", ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", oss3.str().c_str());

  // ===== Publish s_metric và kappa_s =====
  std_msgs::msg::Float64MultiArray s_msg;
  s_msg.data = s_metric;
  std_msgs::msg::Float64MultiArray kappa_s_msg;
  kappa_s_msg.data = kappa_s;
  if (pub_s_) pub_s_->publish(s_msg);
  if (pub_kappa_s_) pub_kappa_s_->publish(kappa_s_msg);

  publish_debug_image(msg, scan_rows, lateral_offset, image_center);
  publish_fitting_image(msg, scan_rows, centroid_x, poly_coeffs, scan_top, scan_bot);
}

// CameraInfo callback
void ImageProcessor::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  // Lấy mảng 9 số thực từ trường k
  const double* k = msg->k.data();
  K_ = (cv::Mat_<double>(3,3) <<
    k[0], k[1], k[2],
    k[3], k[4], k[5],
    k[6], k[7], k[8]);
  has_camera_info_ = true;
  RCLCPP_INFO(this->get_logger(), "[IMG] Đã nhận CameraInfo và cập nhật K");
}

bool ImageProcessor::compute_offsets(
    const unsigned char* data, int width, int height, int step, int channels,
    double image_center, int scan_bot, int span,
    std::vector<double>& lateral_offset, std::vector<int>& scan_rows)
{
    bool any_valid = false;
    for (int i = 0; i < N; i++) {
      double ratio  = static_cast<double>(N - 1 - i) / (N - 1);
      int    center = scan_bot - static_cast<int>(ratio * span);
      scan_rows[i]  = std::max(0, std::min(height - 1, center));

      double sum_mid = 0;
      int cnt = 0;
      for (int r = center - band_height_/2; r <= center + band_height_/2; r++) {
        if (r < 0 || r >= height) continue;
        double midpoint = scan_row(data, r, width, step, channels);
        if (midpoint >= 0) { sum_mid += midpoint; cnt++; }
      }
      if (cnt > 0) {
        lateral_offset[i] = sum_mid / cnt - image_center;
        any_valid = true;
      } else {
        lateral_offset[i] = 0.0;
      }
    }
    return any_valid;
}

bool ImageProcessor::check_valid_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
      RCLCPP_WARN_ONCE(rclcpp::get_logger("ImageProcessor"),
        "Encoding '%s' chưa được hỗ trợ (cần rgb8/bgr8)", msg->encoding.c_str());
      return false;
    }
    return true;
}

int calculate_span(int height, double top_ratio, double bot_ratio) {
    int scan_top = static_cast<int>(height * top_ratio);
    int scan_bot = static_cast<int>(height * bot_ratio);
    return scan_bot - scan_top;
}

// ── Private method: publish_debug_image ───────────────────────────────────
void ImageProcessor::publish_debug_image(
    const sensor_msgs::msg::Image::SharedPtr src,
    const std::vector<int>& rows,
    const std::vector<double>& offsets,
    double image_center)
{
    auto debug = clone_image(src);

    const int width    = static_cast<int>(src->width);
    const int step     = static_cast<int>(src->step);
    const int channels = step / width;
    unsigned char* d   = debug->data.data();

    draw_all_bands(d, src, step, channels, width, rows, offsets, image_center);
    pub_debug_->publish(*debug);
}

void ImageProcessor::draw_all_bands(
    unsigned char* d,
    const sensor_msgs::msg::Image::SharedPtr src,
    int step, int channels, int width,
    const std::vector<int>& rows,
    const std::vector<double>& offsets,
    double image_center)
{
    const uint8_t color[3] = {0, 255, 0}; // xanh lá
    for (int i = 0; i < N; i++) {
      int row_center = rows[i];
      int centroid_x = static_cast<int>(offsets[i] + image_center);

      // Tô 1 dải ngang 3 px xung quanh scan line
      for (int r = row_center - 1; r <= row_center + 1; r++) {
        if (r < 0 || r >= static_cast<int>(src->height)) continue;
        int row_off = r * step;
        for (int x = 0; x < width; x++) {
          int px = row_off + x * channels;
          int brightness = src->data.data()[px];
          if (brightness < black_threshold_) {
            d[px]     = color[0];
            d[px + 1] = color[1];
            d[px + 2] = color[2];
          }
        }
      }
      // Đánh dấu centroid (midpoint) bằng chấm trắng 5x5
      for (int dr = -2; dr <= 2; dr++)
      for (int dc = -2; dc <= 2; dc++) {
        int r = row_center + dr;
        int x = centroid_x  + dc;
        if (r < 0 || r >= static_cast<int>(src->height)) continue;
        if (x < 0 || x >= width) continue;
        int px    = r * step + x * channels;
        d[px]     = 255; d[px+1] = 255; d[px+2] = 255;
      }
    }
} 

sensor_msgs::msg::Image::SharedPtr clone_image(const sensor_msgs::msg::Image::SharedPtr src)
{
    auto img      = std::make_shared<sensor_msgs::msg::Image>();
    img->header   = src->header;
    img->width    = src->width;
    img->height   = src->height;
    img->step     = src->step;
    img->encoding = src->encoding;
    img->data     = src->data;
    return img;
} 
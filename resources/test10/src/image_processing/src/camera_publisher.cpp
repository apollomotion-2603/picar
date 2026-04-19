#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

using std::placeholders::_1;

class CameraSubscriber : public rclcpp::Node {
public:
    CameraSubscriber() : Node("camera_subscriber") {
        cv::namedWindow("Camera | Debug 2x2", cv::WINDOW_NORMAL);
        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vehicle_front_camera/image_raw", 10, std::bind(&CameraSubscriber::camera_callback, this, _1)
);
        sub_debug_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/line/debug_image", 10, std::bind(&CameraSubscriber::debug_callback, this, _1)
        );  
        sub_fitting_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/line/fitting_image", 10, std::bind(&CameraSubscriber::fitting_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "CameraSubscriber node started");
    }

private:
    cv::Mat last_camera_image_;
    cv::Mat last_debug_image_;
    cv::Mat last_fitting_image_;
    cv::Mat last_extra_image_; // Panel thứ 4 (có thể để trống hoặc dùng cho ảnh khác)

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Luôn convert sang bgr8 để OpenCV hiển thị đúng màu
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_camera_image_ = cv_ptr->image.clone();
            show_combined();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (camera): %s", e.what());
        }
    }

    void debug_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert sang bgr8 để đồng nhất với camera image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_debug_image_ = cv_ptr->image.clone();
            show_combined();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (debug): %s", e.what());
        }
    }

    void fitting_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert sang bgr8 để đồng nhất với camera image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_fitting_image_ = cv_ptr->image.clone();
            show_combined();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (fitting): %s", e.what());
        }
    }

    void show_combined() {
        const int W = 320, H = 240;
        cv::Mat panel1, panel2, panel3, panel4;
        // Panel 1: Raw Camera
        if (!last_camera_image_.empty()) {
            cv::resize(last_camera_image_, panel1, cv::Size(W, H));
        } else {
            panel1 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel1, "Waiting camera...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,0), 1);
        }
        // Panel 2: Debug (scan)
        if (!last_debug_image_.empty()) {
            cv::resize(last_debug_image_, panel2, cv::Size(W, H));
            if (panel2.channels() == 1) cv::cvtColor(panel2, panel2, cv::COLOR_GRAY2BGR);
        } else {
            panel2 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel2, "Waiting debug_image...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);
        }
        // Panel 3: Poly Fitting
        if (!last_fitting_image_.empty()) {
            cv::resize(last_fitting_image_, panel3, cv::Size(W, H));
            if (panel3.channels() == 1) cv::cvtColor(panel3, panel3, cv::COLOR_GRAY2BGR);
        } else {
            panel3 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel3, "Waiting fitting_image...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);
        }
        // Panel 4: Extra (có thể để trống hoặc dùng cho ảnh khác)
        if (!last_extra_image_.empty()) {
            cv::resize(last_extra_image_, panel4, cv::Size(W, H));
            if (panel4.channels() == 1) cv::cvtColor(panel4, panel4, cv::COLOR_GRAY2BGR);
        } else {
            panel4 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel4, "Panel 4 (empty)", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200,200,200), 1);
        }
        // Thêm label cho từng panel
        cv::putText(panel1, "Raw Camera",    cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
        cv::putText(panel2, "Debug (scan)",  cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0),   1);
        cv::putText(panel3, "Poly Fitting",  cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,140,255), 1);
        cv::putText(panel4, "Panel 4",       cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200), 1);
        // Ghép 2x2: 640 x 480
        cv::Mat top, bot, combined;
        cv::hconcat(panel1, panel2, top);
        cv::hconcat(panel3, panel4, bot);
        cv::vconcat(top, bot, combined);
        cv::imshow("Camera | Debug 2x2", combined);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_debug_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_fitting_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}

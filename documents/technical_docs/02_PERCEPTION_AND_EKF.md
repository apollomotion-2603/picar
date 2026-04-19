# 02. Nhận diện Làn đường và Ước lượng Trạng thái (EKF)

Tài liệu này chi tiết hóa cách hệ thống "nhìn" và "cảm nhận" trạng thái của nó thông qua Perception Pipeline và EKF.

## 1. Perception Pipeline (Lane Detection)

Quy trình xử lý hình ảnh bao gồm:
1.  **Homography:** Chuyển đổi ảnh Camera sang Bird's Eye View (BEV) để loại bỏ hiện tượng phối cảnh.
2.  **Adaptive Thresholding:** Sử dụng `Gaussian Adaptive Threshold` để tách làn đường trong nhiều điều kiện ánh sáng khác nhau.
3.  **Sliding Window:** Thuật toán cửa sổ trượt để tìm và theo dõi các điểm ảnh thuộc làn trái/phải.
4.  **Cubic Spline Fitting:** 
    - Các điểm ảnh được fit thành đa thức bậc 3: $x_c(s) = as^3 + bs^2 + cs + d$.
    - Đa thức này cho phép tính toán độ cong (curvature) tại bất kỳ điểm nào trong tầm nhìn.

## 2. Thông điệp `LaneState`

Kết quả từ Perception được đóng gói vào message nội bộ:
- `e_y` (n): Sai lệch ngang tại vị trí hiện tại.
- `e_psi` (alpha): Sai lệch góc heading.
- `kappa`: Độ cong tại s = 0.
- `coeff_a, b, c, d`: Các hệ số của đa thức bậc 3.
- `s_max`: Chiều dài tối đa của làn đường tìm thấy.

## 3. Ước lượng Trạng thái (EKF)

Node `ekf_node` đóng vai trò quan trọng trong việc làm mịn dữ liệu:
-   **State Vector:** $[n, alpha, v]$.
-   **Sensor Fusion:** Kết hợp đo đạc từ Camera (n, alpha) và Odometry từ bánh xe (v).
-   **Output:** Cung cấp trạng thái ổn định ngay cả khi Perception bị nhiễu tạm thời.
-   **Safety Reset:** Tự động reset trạng thái về 0 khi xe chuyển từ trạng thái `Disabled` sang `Enabled` để tránh tích lũy sai số khi xe đứng yên.

## 4. Tầm nhìn và Độ ổn định
- **Look-ahead:** Thuật toán Fit đa thức bậc 3 cho phép NMPC "nhìn xa" và biết trước các khúc cua sắp tới thông qua việc nội suy Curvature dọc theo Horizon.
- **Symmetry:** Việc sử dụng BEV chính xác giúp các vạch kẻ đường luôn song song trên ảnh debug, giảm thiểu rung lắc khi điều khiển.

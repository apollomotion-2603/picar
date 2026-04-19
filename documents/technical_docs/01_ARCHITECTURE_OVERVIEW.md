# 01. Hệ thống Xe Tự Hành: Kiến trúc Tổng thể

Tài liệu này mô tả sơ đồ khối và luồng dữ liệu của hệ thống xe tự hành sử dụng NMPC 6 trạng thái.

## 1. Sơ đồ khối (High-Level Architecture)

Hệ thống được thiết kế theo mô hình Modular Pipeline truyền thống:

```
[ Camera ] ──▶ [ Perception ] ──▶ [ EKF ] ──▶ [ NMPC ] ──▶ [ Control ] ──▶ [ Sim/Hardware ]
    ▲               │              │          │              │
    └───────────────┴──────────────┴──────────┴──────────────┘
                         Feedback & State Estimation
```

## 2. Các Node ROS 2 chính

| Node | Vai trò | Input | Output |
| :--- | :--- | :--- | :--- |
| `perception_node` | Nhận diện vạch kẻ đường | `/camera/image` | `/perception/lane_state` |
| `ekf_node` | Ước lượng trạng thái mịn | `lane_state`, `odom` | `/ekf/vehicle_state` |
| `mpc_node` | Bộ điều khiển tối ưu | `lane_state`, `ekf_state` | `/steering_angle`, `/velocity` |
| `visualizer_node` | Hiển thị debug | Đa chủ đề | Debug Image/Markers |
| `vehicle_controller` | Bridge tới Gazebo/Xe thật | `/steering_angle`, `/velocity` | Joint commands |

## 3. Luồng dữ liệu (Data Flow)

1.  **Sensing:** Camera cung cấp hình ảnh RAW.
2.  **Perception:** Chuyển đổi sang Bird's Eye View (BEV), tìm Lane và fit Cubic Polynomial. Kết quả cung cấp `e_y` (n), `e_psi` (alpha) và curvature `kappa`.
3.  **State Estimation (EKF):** Kết hợp perception với Odometry để lọc nhiễu và cung cấp vận tốc `v`, vị trí ngang `n` và góc heading `alpha` ổn định.
4.  **Planning & Control (NMPC):** Sử dụng mô hình Bicycle 6 trạng thái để dự đoán quỹ đạo tối ưu trong tương lai và xuất lệnh điều khiển tới Actuators.

## 4. Hệ tọa độ (Coordinate Frames)

Hệ thống sử dụng hệ tọa độ tham số đường (Path-parametric):
-   **s:** Quãng đường dọc theo đường tham chiếu (Arclength).
-   **n:** Khoảng cách sai lệch ngang (Lateral deviation/CTE).
-   **alpha:** Sai lệch góc hướng (Heading error).

Mô hình này giúp việc bám làn (Lane Following) trở nên tự nhiên hơn so với hệ tọa độ XY toàn cục.

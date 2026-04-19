# 04. Hướng dẫn Quy trình và Cân chỉnh (Workflow & Tuning)

Tài liệu này hướng dẫn cách vận hành hệ thống và các thông số then chốt cần lưu ý khi cân chỉnh (tuning).

## 1. Quy trình Khởi động (Workflow)

Để chạy hệ thống trong môi trường Gazebo:

1.  **Build hệ thống:**
    ```bash
    colcon build --packages-select mpc_pkg lane_msgs perception_pkg ekf_pkg
    source install/setup.bash
    ```

2.  **Chạy Simulation:**
    -   **Map 1 (Oval):** `ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=1`
    -   **Map 3 (DLC):** `ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=3`

3.  **Lưu ý:** Lần đầu chạy sau khi đổi mô hình (ví dụ 4-state sang 6-state), Acados sẽ mất khoảng 30s để biên dịch mã nguồn C. Các lần sau sẽ khởi động tức thì.

## 2. Các thông số Cân chỉnh chính

Các thông số nằm trong file `src/mpc_pkg/config/nmpc.yaml`:

### A. Tốc độ (Velocity)
- `target_v`: Tốc độ mục tiêu khi đi thẳng (nên bắt đầu từ 0.6 - 1.0 m/s).
- `kappa_speed_factor`: Hệ số giảm tốc khi vào cua. Giá trị càng lớn xe càng đi chậm khi cua gắt.
- `v_min_curve`: Tốc độ tối thiểu đảm bảo xe không bị dừng hẳn khi đang ở giữa khúc cua.

### B. Độ chính xác bám làn (Cost Weights)
Nằm trong `acados_settings.py`, biến `Q` (stage cost) và `Qe` (terminal cost):
- **n error weight:** Tăng giá trị này (vd: 50.0) nếu xe bám làn không chặt.
- **alpha error weight:** Tăng giá trị này (vd: 10.0) nếu xe bị hiện tượng "vẩy đuôi" (oscillating).

### C. Giới hạn vật lý (Bounds)
- `delta_max`: Giới hạn góc lái vật lý (thường là 0.61 rad).
- `ddelta_max`: Giới hạn tốc độ bẻ lái (quản lý độ mượt của servo).

## 3. Một số lỗi thường gặp
- **Solver Status 1 (Infeasible):** Xảy ra khi xe văng quá xa khỏi làn đường (n > n_max). Cần tăng trọng số phạt `n` hoặc giảm tốc độ.
- **Perception Latency:** Nếu log báo `timeout`, hãy kiểm tra CPU load hoặc hạ `ctrl_hz` xuống thấp hơn.

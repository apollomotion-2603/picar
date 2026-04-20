# 01. Kiến trúc Tổng thể Hệ thống Xe RC Tự hành

Tài liệu mô tả sơ đồ khối và luồng dữ liệu của hệ thống sử dụng NMPC 6 trạng
thái dựa trên mô hình Spatial Bicycle (Kloeser 2020 mở rộng).

---

## 1. Sơ đồ khối (High-Level Architecture)

Hệ thống theo mô hình Modular Pipeline truyền thống — mỗi khối là một
ROS 2 node độc lập:

```
[ Camera ]                                             [ Sim / Hardware ]
    │                                                         ▲
    ▼                                                         │
[ perception_node ]──┐                                        │
                     │                                        │
                     ▼                                        │
                 /perception/lane_state                       │
                     │                                        │
           ┌─────────┼──────────┐                             │
           ▼         ▼          ▼                             │
      [ ekf_node ]  [ mpc_node ]   [ reset_node ]             │
           │              │              │                    │
           ▼              │              ▼                    │
  /ekf/vehicle_state ─────┤        /car_enabled               │
                          ▼                                    │
              /steering_angle, /velocity ───────►[ vehicle_controller (C++) ]
```

- Tất cả nodes chạy trên ROS 2 Humble, DOMAIN_ID=42.
- Tốc độ: camera 30 Hz, EKF 100 Hz, NMPC 20 Hz.
- Trong sim, `vehicle_controller` là plugin C++ trong Gazebo Fortress.
- Trong hardware, `vehicle_controller` thay bằng firmware Nucleo F411RE.

---

## 2. Các Node ROS 2 chính

| Node | Vai trò | Input chính | Output chính |
|------|---------|-------------|--------------|
| `perception_node` | Nhận diện làn đường | `/camera/image_raw` | `/perception/lane_state` |
| `ekf_node` | Ước lượng trạng thái mịn | lane_state, IMU, joint_states | `/ekf/vehicle_state` |
| `mpc_node` | NMPC 6-state @ 20 Hz | lane_state, ekf_state, car_enabled | `/steering_angle`, `/velocity` |
| `reset_node` | Khởi động/dừng/reset xe | keyboard / service / auto | `/car_enabled`, teleport xe |
| `visualizer_node` | Debug 4-panel | camera | debug images |
| `vehicle_controller` | Bridge sim ↔ NMPC | /steering_angle, /velocity | Joint commands |

---

## 3. Luồng dữ liệu (Data Flow)

1. **Sensing** — Camera (30 Hz), IMU (~77 Hz), joint_states (100 Hz).
2. **Perception** — BEV warp + adaptive threshold + sliding window + cubic
   arc-length polynomial. Output `e_y`, `e_psi`, `κ`, hệ số đa thức.
3. **State Estimation (EKF)** — fuse perception (n, α) + IMU (α̇) + wheel
   encoders (v). Output state mượt @ 100 Hz.
4. **Control (NMPC 6-state)** — dùng mô hình bicycle spatial 6 trạng thái
   để dự đoán quỹ đạo tối ưu 1 giây phía trước (N=20 stages × dt=50 ms)
   rồi xuất `δ` và `v` tại stage 1.
5. **Actuation** — `vehicle_controller` (sim) hoặc Nucleo firmware (HW)
   chuyển `/steering_angle` + `/velocity` thành PWM servo/ESC.

---

## 4. Hệ tọa độ (Coordinate Frames)

Hệ thống sử dụng hệ toạ độ **tham số đường (Path-parametric)** thay vì
XY toàn cục:

| Symbol | Ý nghĩa | Đơn vị |
|--------|---------|--------|
| s | Arc-length dọc đường tham chiếu | m |
| n | Sai lệch ngang (Cross-Track Error) | m |
| α | Sai lệch góc hướng (Heading Error) | rad |
| v | Vận tốc dọc xe | m/s |
| D | Duty cycle throttle | [-1, 1] |
| δ | Góc lái | rad |

Mô hình path-parametric giúp bài toán bám làn (lane following) trở nên tự
nhiên — `n=0, α=0` chính là "đi giữa lane, thẳng hướng lane" — và tránh
singularity tại các khúc cua khi công thức hóa chuẩn.

---

## 5. Phần mềm và Công cụ

| Thành phần | Version | Ghi chú |
|------------|---------|---------|
| ROS 2 | Humble | native laptop, Docker Pi |
| Gazebo | Ignition 6 / Fortress | lệnh `ign`, msg `ignition.msgs.*` |
| acados | v0.3.5 | `~/acados/`, build từ source |
| tera_renderer | v0.2.0 | Rust, required bởi acados |
| Python | 3.10 | |
| OpenCV | 4.x | perception pipeline |
| NumPy / SciPy | — | fit đa thức, EKF |

---

## 6. Workspace layout

```
ros2_ws/
├── CLAUDE.md                 ← entry AI
├── PROJECT_STATE.md          ← technical index
├── src/
│   ├── perception_pkg/
│   ├── ekf_pkg/
│   ├── mpc_pkg/              ← NMPC 6-state + acados
│   ├── lane_msgs/
│   └── gazebo_ackermann_steering_vehicle/
├── nmpc_build_6state/        ← solver cache (xóa để rebuild)
├── data/bags/                ← rosbag + analyze scripts
└── documents/
    ├── plan/                 ← phase plans
    ├── state/                ← technical state chi tiết
    └── technical_docs/       ← tài liệu này
```

---

## 7. Vì sao chia thành modules

- **Testability:** mỗi node test độc lập bằng `ros2 topic pub` giả sensor.
- **Swap-ability:** đổi perception (BEV → YOLO, chẳng hạn) không ảnh hưởng
  NMPC vì interface là `LaneState` message.
- **Portability sim → hardware:** chỉ `vehicle_controller` đổi backend.
- **Parameter-driven:** mọi tune nằm trong yaml, không hardcode.

Các tài liệu tiếp theo:
- [02_PERCEPTION_AND_EKF.md](02_PERCEPTION_AND_EKF.md) — lane detection + EKF.
- [03_NMPC_CONTROL_LOGIC.md](03_NMPC_CONTROL_LOGIC.md) — 6-state NMPC.
- [04_WORKFLOW_AND_TUNING.md](04_WORKFLOW_AND_TUNING.md) — vận hành + tuning.

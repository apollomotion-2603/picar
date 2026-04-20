# 04. Quy trình vận hành & Cân chỉnh (Workflow & Tuning)

Hướng dẫn cách vận hành hệ thống + các thông số then chốt khi tuning.

---

## 1. Quy trình khởi động (Sim)

### 1.1 Build

```bash
cd ~/main/1_projects/1_autonomous_car_research/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Build packages riêng lẻ nếu cần: `--packages-select mpc_pkg lane_msgs
perception_pkg ekf_pkg gazebo_ackermann_steering_vehicle`.

### 1.2 Xoá NMPC solver cache

Bất cứ khi nào đổi:
- `nmpc.yaml` / `nmpc_map1.yaml` (trong `src/mpc_pkg/config/`)
- `bicycle_model.py`
- `acados_settings.py`

phải:

```bash
rm -rf nmpc_build_6state/
```

Nếu không, solver dùng binary cũ → thay đổi không có tác dụng.

### 1.3 Chạy sim (2 terminal)

```bash
export ROS_DOMAIN_ID=42
source install/setup.bash

# T1 — Full sim (map 1 default)
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=1

# T2 — Reset node (TTY keyboard)
ros2 run ekf_pkg reset_node --ros-args -p map_id:=1
# s=START  x=STOP  q=RESET (teleport)
```

**Lần đầu chạy 6-state model:** acados mất ~30 s compile C code. Các
lần sau dùng cache `nmpc_build_6state/`, khởi động <5 s.

### 1.4 Record rosbag + phân tích

```bash
# T3 — record
cd data/bags
ros2 bag record -o bag_map_test_N \
  /perception/lane_state /ekf/vehicle_state \
  /steering_angle /velocity /car_enabled /tf

# Analyze
python3 data/bags/analyze_bag7.py data/bags/bag_map_test_N
```

Script in stats theo section (early/middle/late) + δ sign flips để
phát hiện oscillation.

---

## 2. Các thông số cân chỉnh (NMPC)

### 2.1 Vận tốc (`nmpc.yaml`)

| Param | Default | Ý nghĩa |
|-------|---------|---------|
| `target_v` | 1.0 | Tốc độ mục tiêu đường thẳng (thử 0.6–1.2) |
| `kappa_speed_factor` | 2.0 | Hệ số giảm tốc khi cua (càng lớn càng chậm khi cua gắt) |
| `v_min_curve` | 0.3 | Tốc độ tối thiểu trong cua — tránh stall giữa khúc cua |
| `v_max` | 1.5 | Hard bound, không vượt ngay cả khi cost muốn cao hơn |

Công thức: $v_{ref} = \text{clip}(\frac{v_{target}}{1+k_f|\kappa|},\ v_{min\_curve},\ v_{max})$.

### 2.2 Cost weights (`acados_settings.py`)

Hardcoded trong file (không qua yaml hiện tại):

- **Tăng weight `n`** (currently 50) nếu xe bám không chặt.
- **Tăng weight `α`** (currently 10) nếu xe "vẩy đuôi" oscillation.
- **Giảm weight `derDelta`** (currently 5e-3) để steering nhanh hơn, hoặc
  tăng để mượt hơn nhưng phản ứng chậm.
- **Tăng weight `v`** (currently 0.1) nếu tracking `v_ref` kém — nhưng
  cẩn thận mất ưu tiên `n, α`.

Sau khi đổi cost → `rm -rf nmpc_build_6state/` rồi build lại.

### 2.3 Curvature handling

| Param | Map 1 | Map 3 | Ghi chú |
|-------|-------|-------|---------|
| `kappa_max` | 2.0 | **0.8** | Clamp κ trong horizon — chặn perception outlier |
| `kappa_ema_alpha` | 0.5 | **0.7** | EMA smoothing (càng lớn càng smooth) |

Map 3 DLC perception noisy hơn → clamp chặt + smooth mạnh hơn.

### 2.4 Giới hạn vật lý (`nmpc.yaml`)

| Param | Value | Ý nghĩa |
|-------|-------|---------|
| `delta_max` | 0.6109 rad (~35°) | Giới hạn cơ khí servo |
| `ddelta_max` | 2.0 rad/s | Tốc độ bẻ lái max (mượt servo) |
| `throttle_min/max` | -1.0 / +1.0 | Duty cycle ESC |
| `dthrottle_max` | 10.0 /s | Rate limit throttle |
| `n_max` | 0.20 m | Soft bound lateral offset |
| `alat_max` / `along_max` | 4.0 m/s² | Soft accel bounds |

---

## 3. Thông số Perception (`perception.yaml`)

### 3.1 BEV homography (phải recalibrate khi đổi camera/track)

```yaml
bev_src: [x1, y1, x2, y2, x3, y3, x4, y4]  # TL TR BR BL pixel
bev_dst: [100, 0, 300, 0, 300, 500, 100, 500]
bev_width: 400
bev_height: 500
bev_scale: 0.0025   # m/px — lane 50cm = 200px
```

⚠️ **Phải sync tay** vào `visualizer_node.py:17-18` (hardcoded SRC).

### 3.2 Sliding window & threshold

| Param | Default | Tuning |
|-------|---------|--------|
| `n_windows` | 10 | nhiều hơn → tracking chi tiết nhưng tốn CPU |
| `win_width` | 100 | tăng nếu lane detect bị mất lúc DLC |
| `min_pixels` | 20 | giảm nếu vạch mờ; tăng để lọc false detect |
| `thresh_block_size` | 35 | kernel adaptive thresh (odd) |
| `thresh_c` | 10 | offset thresh — tăng nếu nền sáng |
| `s_max_m` | 0.7 | camera lookahead |

---

## 4. Debugging thường gặp

### 4.1 NMPC

| Symptom | Nguyên nhân có khả năng | Giải pháp |
|---------|-------------------------|-----------|
| Solver status 1 (infeasible) | Xe lệch quá `n_max` | Tăng slack, giảm `target_v`, kiểm tra `lane_detected` |
| `Gauss-Newton + EXTERNAL` warn | Acados bình thường | Bỏ qua |
| Xe dao động đường thẳng | κ bias từ perception khi xe đứng yên | Fix đã apply: reset κ=0 khi enable |
| DLC đoạn cuối văng ra | κ spike từ cubic overfit | Clamp `kappa_max=0.8`, smooth EMA |
| Stall giữa cua (v→0) | `v_ref` → 0 khi κ lớn | Tăng `v_min_curve` |
| Override v không khớp cost | (đã fix) | Hướng A: không override tại publish |

### 4.2 Perception

| Symptom | Giải pháp |
|---------|-----------|
| Lane detect nhảy, kappa noisy | Tăng `kappa_ema_alpha`, giảm `win_width` |
| Chữ "V" trong LANES panel | Near-field weighting (đã apply), giảm `s_max_m` |
| κ sign sai qua S-curve | Kiểm tra BEV calib; lane markings phải song song |
| `lane_detected=False` kéo dài | Tune `thresh_c`, ánh sáng đều |

### 4.3 Gazebo

| Symptom | Giải pháp |
|---------|-----------|
| Segfault lúc load | KHÔNG add `ign-gazebo-sensors-system` vào world SDF |
| `/imu/data` không có data | Kiểm tra `ignition-gazebo-imu-system` plugin trong xacro |
| Teleport không phản hồi | Dùng `ign service` (không `gz service`), format protobuf `:` |

---

## 5. Tuning workflow đề xuất

1. **Fix bằng cách thêm log** — xem `κ₀, vref₀, v, δ` mỗi tick.
2. **Record bag** trước và sau khi đổi tham số → so sánh.
3. **Đổi 1 tham số một lần** — ghi lại kết quả.
4. **Build → sim → bag → analyze** = 1 iteration (~2 phút).
5. Khi đã stable sim → chuyển sang hardware với `target_v=0.3`
   và cùng cost weights (scale sẽ cần đổi nhẹ do dynamics thật khác).

---

## 6. Files cần quan tâm khi tune

| File | Vai trò |
|------|---------|
| `src/mpc_pkg/config/nmpc.yaml` | NMPC params (map 3) |
| `src/mpc_pkg/config/nmpc_map1.yaml` | NMPC params (map 1) |
| `src/mpc_pkg/mpc_pkg/acados_settings.py` | Cost weights Q/R/Qe |
| `src/mpc_pkg/mpc_pkg/bicycle_model.py` | Mô hình động lực học |
| `src/perception_pkg/config/perception.yaml` | BEV, sliding window |
| `src/ekf_pkg/ekf_pkg/ekf_node.py` | EKF gains (Q, R covariance) |

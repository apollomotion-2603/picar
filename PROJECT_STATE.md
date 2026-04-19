# PROJECT STATE — RC Car Autonomous Driving
# apollomotion (Phi) | ĐH Giao thông Vận tải TP.HCM | Updated: 2026-04-16

---

## 1. PROJECT OVERVIEW

**Mô tả:** Xe RC tự hành 1/16 scale làm luận văn tốt nghiệp. Stack: ROS2 Humble +
Ignition Gazebo 6 (Fortress) + acados NMPC. Pipeline: camera → lane detection → EKF → NMPC → servo/ESC.

**Phân công:**
- **Phi (apollomotion):** Perception, EKF, SLAM, Nav2, GUI, luận văn
- **Nhã:** NMPC optimization, velocity control, Nucleo firmware

### Hardware Specs

| Parameter     | Value      | Note                    |
|---------------|------------|-------------------------|
| m             | 2.5 kg     | body + 4 wheels         |
| lf = lr       | 0.11 m     | Symmetric               |
| L (wheelbase) | 0.22 m     | lf + lr                 |
| wheel_radius  | 0.04 m     |                         |
| body (L×W×H)  | 0.30×0.18×0.05 m |                   |
| max_steering  | 0.6109 rad | ≈ 35°                   |
| lane_width    | 0.50 m     |                         |
| V_max (sim)   | 1.5 m/s    | hardware phase: 0.3 m/s |

### Software Stack

| Component       | Version / Details                                  |
|-----------------|----------------------------------------------------|
| OS              | Ubuntu 22.04 (laptop), RPi OS 64-bit (Pi)          |
| ROS2            | Humble (native on laptop, Docker on Pi)            |
| Gazebo          | Ignition Gazebo 6 / Fortress (`ign` CLI)           |
| acados          | v0.3.5 at `~/acados/`                              |
| tera_renderer   | v0.2.0 (built from source Rust)                    |
| ACADOS_SOURCE_DIR | `~/acados` (set in mpc_node.py line 17)          |
| GPU             | NVIDIA Quadro T1000 4GB                            |
| ROS_DOMAIN_ID   | 42                                                 |

### Workspace

```
~/main/1_projects/1_autonomous_car_research/ros2_ws/
```

---

## 2. PACKAGE STRUCTURE

| Package | Description | Nodes |
|---------|-------------|-------|
| `perception_pkg` | Camera → BEV → lane detection → LaneState | `perception_node`, `visualizer_node`, `grid_viewer` |
| `mpc_pkg` | NMPC lane following (Kloeser 2020) via acados | `mpc_node` |
| `ekf_pkg` | EKF sensor fusion + reset/keyboard control | `ekf_node`, `reset_node` |
| `lane_msgs` | Custom ROS2 message definitions | — (msgs only) |
| `gazebo_ackermann_steering_vehicle` | Sim world, URDF/xacro, vehicle controller, launch files | `vehicle_controller` (C++) |

### Node Details

**`perception_node`** (`perception_pkg/perception_node.py`)
- Sub: `/camera/image_raw`
- Pub: `/perception/lane_state`
- Reads params from `perception.yaml`

**`visualizer_node`** (`perception_pkg/visualizer_node.py`)
- Sub: `/camera/image_raw`
- Pub: `/perception/view1_raw`, `/perception/view2_bev`, `/perception/view3_threshold`,
  `/perception/view4_lanes`, `/perception/debug_grid`
- **IMPORTANT:** SRC/DST BEV points are **hardcoded** (not from yaml). Must update manually if perception.yaml changes.

**`grid_viewer`** (`perception_pkg/grid_viewer.py`)
- Sub: `/perception/debug_grid`
- Displays 2×2 debug grid via `cv2.imshow()` (GUI window "Perception Debug [2x2]" at 1280×1040)

**`mpc_node`** (`mpc_pkg/mpc_node.py`)
- Sub: `/perception/lane_state`, `/ekf/vehicle_state`, `/car_enabled`
- Pub: `/steering_angle`, `/velocity`
- Reads params from `nmpc.yaml`; builds acados solver into `nmpc_build/` on first run (~5s)

**`ekf_node`** (`ekf_pkg/ekf_node.py`)
- Sub: `/imu/data`, `/perception/lane_state`, `/joint_states`
- Pub: `/ekf/vehicle_state`
- State: `[n, ṅ, alpha, alpha_dot, v, psi]`

**`reset_node`** (`ekf_pkg/reset_node.py`)
- Sub: `/perception/lane_state`
- Pub: `/velocity`, `/steering_angle`, `/car_enabled`
- Service: `/reset_car` (std_srvs/Trigger)
- Keyboard (stdin TTY): `s`=START, `x`=STOP, `q`=RESET
- Auto-reset when `lane_detected=False` for > 3.0 seconds (while car_enabled=True)
- Param: `map_id` (int, default=1)

---

## 3. ROS2 INTERFACES

### Topics

| Topic | Type | Publisher | Subscriber(s) | Hz |
|-------|------|-----------|---------------|----|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge | `perception_node`, `visualizer_node` | 30 |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo bridge | `ekf_node` | ~77 |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo bridge | `ekf_node` | 100 |
| `/perception/lane_state` | `lane_msgs/LaneState` | `perception_node` | `mpc_node`, `ekf_node`, `reset_node` | 30 |
| `/perception/debug_grid` | `sensor_msgs/Image` | `visualizer_node` | `grid_viewer` | 30 |
| `/perception/view1_raw` | `sensor_msgs/Image` | `visualizer_node` | — | 30 |
| `/perception/view2_bev` | `sensor_msgs/Image` | `visualizer_node` | — | 30 |
| `/perception/view3_threshold` | `sensor_msgs/Image` | `visualizer_node` | — | 30 |
| `/perception/view4_lanes` | `sensor_msgs/Image` | `visualizer_node` | — | 30 |
| `/ekf/vehicle_state` | `lane_msgs/VehicleState` | `ekf_node` | `mpc_node` | 100 |
| `/car_enabled` | `std_msgs/Bool` | `reset_node` | `mpc_node` | 10 |
| `/steering_angle` | `std_msgs/Float64` | `mpc_node`, `reset_node` | vehicle_controller | 30 |
| `/velocity` | `std_msgs/Float64` | `mpc_node`, `reset_node` | vehicle_controller | 30 |

### Services

| Service | Type | Server | Description |
|---------|------|--------|-------------|
| `/reset_car` | `std_srvs/Trigger` | `reset_node` | Teleport xe về spawn point, stop car |

### Custom Messages

**`lane_msgs/LaneState`** (published @ 30Hz by `perception_node`):
```
std_msgs/Header header
float64 e_y        # lateral offset [m] = n in NMPC (= coeff_d from poly fit)
float64 e_psi      # heading error [rad] = alpha in NMPC (= arctan(coeff_c))
float64 kappa      # curvature [1/m] = 2b / (1 + c²)^1.5
float64 coeff_a    # cubic poly: x_c(s) = a*s³ + b*s² + c*s + d
float64 coeff_b
float64 coeff_c
float64 coeff_d
float64 s_max      # camera lookahead [m]
bool lane_detected
```

**`lane_msgs/VehicleState`** (published @ 100Hz by `ekf_node`):
```
std_msgs/Header header
float64 n           # lateral offset [m]
float64 n_dot       # lateral velocity [m/s]
float64 alpha       # heading error [rad]
float64 alpha_dot   # heading rate [rad/s]
float64 v           # forward velocity [m/s]
float64 psi         # absolute heading [rad]
float64 std_n       # 1-sigma uncertainty [m]
float64 std_v       # 1-sigma uncertainty [m/s]
float64 std_alpha   # 1-sigma uncertainty [rad]
bool ekf_healthy
```

---

## 4. LAUNCH COMMANDS

### Full Pipeline (1 command)

```bash
export ROS_DOMAIN_ID=42
source ~/main/1_projects/1_autonomous_car_research/ros2_ws/install/setup.bash

# Terminal 1 — Full sim (Map 1 mặc định)
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py
# Hoặc map khác: map:=2 / map:=3 / map:=4

# Terminal 2 — Reset node (chạy riêng để có TTY keyboard)
ros2 run ekf_pkg reset_node --ros-args -p map_id:=1
# s=START  x=STOP  q=RESET (teleport về spawn)
```

**Startup sequence (từ `sim_full_launch.py`):**

| Delay | Action |
|-------|--------|
| 0s | Ignition Gazebo + spawn + robot_state_publisher + ros_gz_bridge + vehicle_controller |
| After spawn exits | joint_state_broadcaster activated |
| After joint_state_broadcaster | forward_velocity_controller + forward_position_controller activated |
| +5s | `perception_node` starts |
| +6s | `ekf_node` starts |
| +6.5s | `visualizer_node` starts |
| +7s | `mpc_node` starts (builds solver ~5s on first run) |
| +7.5s | `grid_viewer` starts |
| **reset_node KHÔNG còn trong launch** — chạy riêng terminal 2 |

### Individual Nodes (manual multi-terminal)

```bash
# Terminal 1: Gazebo
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py \
  world:=$(ros2 pkg prefix gazebo_ackermann_steering_vehicle)/share/gazebo_ackermann_steering_vehicle/worlds/lane_track.sdf \
  x:=1.5 y:=1.0

# Terminal 2: Perception
ros2 run perception_pkg perception_node --ros-args --params-file \
  src/perception_pkg/config/perception.yaml

# Terminal 3: EKF
ros2 run ekf_pkg ekf_node

# Terminal 4: NMPC
ros2 run mpc_pkg mpc_node --ros-args --params-file \
  src/mpc_pkg/config/nmpc.yaml

# Terminal 5: Reset node
ros2 run ekf_pkg reset_node
# Hoặc map 2:
ros2 run ekf_pkg reset_node --ros-args -p map_id:=2
```

### Reset / Control Commands

```bash
# Keyboard (khi reset_node chạy trong terminal TTY):
#   s + Enter → START xe
#   x + Enter → STOP xe
#   q + Enter → RESET xe về spawn point

# Service call reset (khi reset_node không có TTY):
ros2 service call /reset_car std_srvs/srv/Trigger

# Debug visualizer
rqt_image_view /perception/debug_grid
```

### Build Commands

```bash
cd ~/main/1_projects/1_autonomous_car_research/ros2_ws

# Build tất cả
colcon build --symlink-install

# Build từng package
colcon build --packages-select lane_msgs --symlink-install
colcon build --packages-select perception_pkg --symlink-install
colcon build --packages-select ekf_pkg --symlink-install
colcon build --packages-select mpc_pkg --symlink-install
colcon build --packages-select gazebo_ackermann_steering_vehicle --symlink-install

# Source
source install/setup.bash

# Xóa NMPC solver cache (BẮT BUỘC khi thay đổi nmpc.yaml/model)
rm -rf nmpc_build

# Fix build conflicts
rm -rf build/lane_msgs install/lane_msgs
```

---

## 5. PERCEPTION PIPELINE

**File:** `src/perception_pkg/perception_pkg/perception_node.py`
**Config:** `src/perception_pkg/config/perception.yaml`

### Processing Chain

```
/camera/image_raw (640×480, BGR)
  │
  ├─ BEV warp: cv2.warpPerspective(img, H, (BEV_W=300, BEV_H=500))
  │    H = cv2.getPerspectiveTransform(SRC, DST)
  │
  ├─ cv2.cvtColor(BGR → GRAY)
  ├─ cv2.GaussianBlur(5, 5)
  ├─ cv2.adaptiveThreshold(ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV,
  │    blockSize=35, C=10)
  │
  ├─ Sliding Window (N_WIN=10 windows, WIN_W=60px, MIN_PIX=20)
  │    └─ histogram peak → left_x, right_x initial → slide bottom-to-top
  │
  ├─ Center points = midpoint(left_pts, right_pts)   [need ≥4 per side]
  │
  ├─ Arc-length cubic fit (fit_cubic_arclength):
  │    pixel coords → metric coords (scale=0.0025 m/px)
  │    polyfit(u_dense, x_center, deg=3) in u-space
  │    reparameterize by arc-length s
  │    polyfit(s_dense, x_center, deg=3) → coeff [a, b, c, d]
  │
  └─ Publish /perception/lane_state:
       e_y   = coeff_d          (lateral offset at s=0)
       e_psi = arctan(coeff_c)  (heading error at s=0)
       kappa = 2b / (1 + c²)^1.5  (curvature at s=0)
```

### BEV Homography Parameters (SIM — recalibrate for hardware)

```yaml
# perception.yaml
bev_src: [172.0, 151.0, 471.0, 151.0, 609.0, 365.0, 29.0, 368.0]
# SRC corners (pixel): TL, TR, BR, BL in camera image

bev_dst: [100.0, 0.0, 300.0, 0.0, 300.0, 500.0, 100.0, 500.0]
# DST corners (pixel): lane 200px căn giữa BEV 400px (100→300)

bev_width: 400    # Thu hẹp: lane 200px chiếm 50% → ít noise hơn
bev_height: 500   # BEV output height [px]
bev_scale: 0.0025 # m/px — lane 0.5m = 200px
```

### Sliding Window Parameters

```yaml
n_windows: 10     # number of windows
win_width: 100    # window width [px] (tăng 80→100 giúp tracking khoẻ hơn khi chuyển làn DLC)
min_pixels: 20    # minimum pixels to recenter window (giảm false detect)
```

### Arc-Length Parameters

```yaml
s_max_m: 0.7      # camera lookahead [m] (giảm 1.0→0.7: 2.3× thân xe, ít kappa drift)
n_arc: 200        # number of arc-length integration points
thresh_block_size: 35
thresh_c: 10
```

### Validated Results (sim, xe đứng yên trên đường thẳng)

- `e_y ≈ -0.001 m`, `e_psi ≈ -0.018 rad`, `kappa ≈ 0.059 1/m`
- Correct kappa sign-change qua S-curve ✅

---

## 6. NMPC CONFIGURATION

**File:** `src/mpc_pkg/mpc_pkg/mpc_node.py`
**Config:** `src/mpc_pkg/config/nmpc.yaml`
**Reference:** Kloeser 2020 — "NMPC for Racing Using a Singularity-Free Path-Parametric Model"

### Model

```
State:     x = [s, n, alpha, v]
Control:   u = [delta, v_ref]
Parameter: kappa_c  (online, from /perception/lane_state)

Kinematics (singularity-free):
  beta    = (lr / L) * delta          # side-slip angle
  s_dot   = v * cos(alpha + beta) / (1 - n * kappa_c)
  n_dot   = v * sin(alpha + beta)
  a_dot   = (v / lr) * sin(beta) - kappa_c * s_dot
  v_dot   = (v_ref - v) / tau_v
```

### Solver Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| N | 30 | Prediction horizon steps |
| Tf | 1.0 s | Prediction horizon time |
| ctrl_hz | 30 Hz | Control loop rate |
| qp_solver | PARTIAL_CONDENSING_HPIPM | |
| hessian_approx | GAUSS_NEWTON | |
| integrator_type | ERK | Explicit Runge-Kutta |
| nlp_solver_type | SQP_RTI | Real-Time Iteration |
| Build dir | `nmpc_build/` | Cached; delete to rebuild |
| Build time | ~5s first run | |
| Solve time | 1–5 ms @ 30Hz | ✅ |

### Cost Functions (EXTERNAL type)

**Stage cost** (nodes 0..N-1):
```
L(x,u) = w_n * n²  +  w_alpha * alpha²  +  w_delta * delta²  +  w_v * (v - v_ref)²
```

**Terminal cost** (node N):
```
L_e(x) = -w_s_e * s  +  w_n_e * n²  +  w_a_e * alpha²
```

### Cost Weights

| Weight | Value | Description |
|--------|-------|-------------|
| w_n | 200.0 | Lateral offset penalty |
| w_alpha | 100.0 | Heading error penalty |
| w_delta | **80.0** | Steering magnitude penalty (tăng 50→80 để suppress DLC oscillation; `rm -rf nmpc_build/`) |
| kappa_max | **3.0** | Clamp kappa_pred trong horizon (1/m) — spike > 3 là noise, ngăn singularity |
| diagnostic_kappa_zero | **false** | Debug flag: true = bypass kappa từ perception (test isolate MPC vs perception) |
| w_v | 0.5 | Velocity tracking penalty |
| w_s_e | 50.0 | Progress reward (terminal) |
| w_n_e | 2000.0 | Lateral offset penalty (terminal) |
| w_a_e | 5000.0 | Heading error penalty (terminal) |

### Constraints

| Variable | Lower | Upper | Note |
|----------|-------|-------|------|
| n (state idx 1) | -0.20 m | +0.20 m | n_max |
| alpha (state idx 2) | -0.6109 rad | +0.6109 rad | delta_max applied |
| v (state idx 3) | 0.0 m/s | 2.0 m/s | v_min / v_max |
| delta (control idx 0) | -0.6109 rad | +0.6109 rad | ≈ ±35° |
| v_ref (control idx 1) | 0.0 m/s | v_ref_adaptive | upper bound adaptive |

### Adaptive Speed Formula

```python
v_ref_adaptive = V_MAX / (1.0 + kappa_speed_factor * abs(kappa))
v_ref_adaptive = clip(v_ref_adaptive, v_min_curve=0.25, V_MAX=1.5)
```

### Velocity Source Logic

```python
# n, alpha: dùng EKF nếu healthy (filtered), fallback sang raw perception
if ekf_healthy and ekf_n is not None:
    x_est[1] = clip(ekf_n,     -N_MAX,     N_MAX)
    x_est[2] = clip(ekf_alpha, -DELTA_MAX, DELTA_MAX)
else:
    x_est[1] = perc_n
    x_est[2] = perc_alpha

# v: EKF hoặc first-order model
if ekf_healthy and ekf_v is not None:
    x_est[3] = clip(ekf_v, V_MIN, V_MAX)       # use EKF feedback
else:
    x_est[3] += (v_cmd - v) / tau_v * dt        # first-order model fallback
```

### Safety Logic

- If `/car_enabled` = False → publish (delta=0, v=0), skip solve
- If lane timeout > `timeout=1.0s` → publish (delta=0, v=0), skip solve
- Solver status not in [0, 2] → reuse previous u (warn log)

### Log Format

```
n=XXXmm α=X.XX° v=X.XXm/s(EKF|model) δ=X.XX° κ=X.XXX v_ref=X.XX t=X.Xms
```

---

## 7. MAP CONFIGURATIONS

### Summary Table

| Map | Arg | World SDF | Spawn x | Spawn y | Spawn yaw | Reset x | Reset y | Reset yaw |
|-----|-----|-----------|---------|---------|-----------|---------|---------|-----------|
| 1 | `map:=1` | `lane_track.sdf` | 1.5 | 1.0 | 0.0 | 0.0 | 1.0 | 0.0 |
| 2 | `map:=2` | `track_test.sdf` | 0.0 | -2.666 | 0.0 | 0.0 | -2.666 | 0.0 |
| 3 | `map:=3` | `lane_change.sdf`| 0.54 | 0.75 | 0.0 | 0.54 | 0.75 | 0.0 |
| 4 | `map:=4` | `obstacle_track.sdf`| 1.5 | 1.0 | 0.0 | 0.0 | 1.0 | 0.0 |

> **Note:** Spawn position (from `sim_full_launch.py`) and reset position (from `reset_node.py` MAP_CONFIGS) differ for Map 1:
> - Spawn: x=1.5, y=1.0 (on the oval track)
> - Reset: x=0.0, y=1.0

### Map Descriptions

**Map 1 — `lane_track.sdf`:** Oval track, 9×2m. Simple closed loop, no chicane.
Lane texture: `lane_track.png` (4096×910px).

**Map 2 — `track_test.sdf`:** Complex track, 6×6m. Chicane + S-curve.
Lane texture: `track_test.png` (2048×2048px).
Spawn angle: Y=0.0 (updated — old value in CLAUDE.md was -1.5708, now 0.0 in launch file).

**Map 3 — `lane_change.sdf`:** Double lane change track, 8×2m.
Lane texture: `lane_change.png`. Đã test thành công NMPC tự động đổi làn.

**Map 4 — `obstacle_track.sdf`:** Dựa trên map 1, bổ sung vật cản tĩnh.
Chuẩn bị cho Phase 3: Obstacle Avoidance.

### Reset Teleport Command (internal)

```bash
# ĐÚNG — Ignition Gazebo 6 (Fortress): dùng ign, không phải gz
ign service -s /world/{world_name}/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "ackermann_steering_vehicle" position: {x: X y: Y z: 0.05} orientation: {x: 0 y: 0 z: QZ w: QW}'
# Lưu ý protobuf text format cho Ignition Fortress:
#   - PHẢI có dấu ":" trước nested message: position: {x: ...}  ← ĐÚNG
#   - KHÔNG dùng format mới: position { x: ... }                 ← SAI (parse fail trên Fortress)
#   - KHÔNG dùng dấu phẩy giữa các fields trong {}
```

---

## 8. CURRENT STATUS

### Phase Progress

| Phase | Description | Status |
|-------|-------------|--------|
| Phase 1 (Tuần 1-4) | Simulation — Perception + NMPC + EKF | ✅ DONE |
| Phase 1.5 (Tuần 5-6) | Deploy hardware — Lane following thực tế | 🔄 IN PROGRESS |
| Phase 2 (Tuần 7-8) | SLAM + Localization (camera-based) | 📅 PLANNED |
| Phase 3 (Tuần 9-10) | Navigation + Obstacle avoidance (Nav2) | 📅 PLANNED |
| Phase 4 (Tuần 11) | GUI Dashboard (Foxglove) | 📅 PLANNED |
| Phase 5 (Tuần 12) | Luận văn + Demo live | 📅 PLANNED |

### Deadlines

| Milestone | Date |
|-----------|------|
| Rough demo / draft | April 18, 2026 (TODO: verify) |
| Final submission | April 25, 2026 (TODO: verify) |

### Completed (Phase 1 + Sim Smoothing)

- [x] Ignition Gazebo 6 (Fortress) sim environment (oval + complex tracks)
- [x] Camera-based BEV lane detection with cubic arc-length polynomial
- [x] Sliding window algorithm with adaptive threshold
- [x] EKF fusion: IMU predict + camera update [n, alpha] + joint_states update [v]
- [x] NMPC (Kloeser 2020) — acados RTI+HPIPM — closed loop @ 30Hz
- [x] Adaptive speed from curvature
- [x] EKF n/alpha/v feedback cho NMPC initial state (giảm dao động lái)
- [x] Reset node: keyboard + service + auto-stop 2s + teleport qua `ign service`
- [x] Full pipeline launch file (`sim_full_launch.py`), reset_node chạy riêng terminal 2
- [x] Visualizer: 4-panel debug grid
- [x] Fix joint damping + body mass → tốc độ thực tế khớp v_cmd
- [x] BEV width thu hẹp 700→400px (lane 50% BEV), win_width 100, min_pixels 20
- [x] mpc_node: clip kappa predict trong horizon ≤ perc_s_max → tránh cubic poly extrapolate vô nghĩa
- [x] mpc_node: reset x_est[3] về EKF/0 khi car_enabled False→True → bỏ jerk lúc start
- [x] visualizer_node: đọc tất cả BEV params từ yaml (không còn hardcode)
- [x] visualizer_node: overlay LANE_STATE thật (e_y, e_psi, kappa) từ MPC lên GUI (xanh lá)
- [x] ekf_node + mpc_node: Thêm EMA Kappa đúng tỉ lệ (0.7 old + 0.3 new) — đã fix lại từ 0.2/0.8 sai
- [x] ekf_node: Thêm EMA kappa trước khi dùng trong EKF predict alpha_dot (tránh noise → EKF diverge)
- [x] reset_node: Fix auto-stop timer, dùng N_MAX_STOP=0.22 để tránh false trigger trong DLC
- [x] **w_delta tăng 50 → 80** để suppress DLC oscillation (`rm -rf nmpc_build/` cần thiết)
- [x] **kappa_max=3.0**: clamp kappa_pred trong horizon — ngăn spike → singularity `1-n*kappa_c`
- [x] **diagnostic_kappa_zero flag** trong nmpc.yaml: bypass kappa từ perception để isolate bug
- [x] **Polynomial bowing fix** (perception_node.py `fit_cubic_arclength`):
  - Clip `us_m ≤ s_max_m` trước polyfit — loại bỏ điểm xa bị sliding window drift
  - Near-field exponential weighting `exp(-s/half)` cho cả 2 lần polyfit
  - **kappa từ quadratic fit (bậc 2)** thay vì cubic — ít overfit, không còn gây xe văng
  - coeff_a,b,c,d vẫn từ cubic (dùng cho MPC horizon tracking)

### Kappa Pipeline Overhaul (Apr 2026) — Fix dao động lái

Phân tích hệ thống toàn bộ pipeline kappa từ perception → EKF → MPC. Phát hiện 7 vấn đề, fix 4 vấn đề quan trọng nhất:

- [x] **Fix #1 — zip ghép sai Y (perception_node)**: `sliding_window()` trả thêm `center_pts`
  chỉ chứa center từ windows có CẢ HAI lane detect cùng Y. Trước đó `zip(left_pts, right_pts)`
  ghép theo index → khi window miss không đều giữa 2 bên → center points ở Y khác nhau →
  polynomial fit sai → kappa/e_psi sai. Fix này làm map 3 DLC cải thiện rõ rệt.

- [x] **Fix #2 — Thống nhất kappa source (mpc_node)**: Horizon kappa giờ dùng `kappa_smooth`
  (quadratic fit + EMA, cùng nguồn với perception publish). Trước đó MPC re-derive kappa
  từ cubic coefficients → hai kappa khác nhau tại s=0. Constant kappa toàn horizon là
  hợp lý vì camera chỉ nhìn 0.7m, control rate 30Hz, và `kappa=0` đã chạy tốt.

- [x] **Fix #3 — Bỏ EMA trên polynomial coefficients (mpc_node)**: Xóa hoàn toàn `poly_a/b/c/d`
  và EMA của chúng. Trộn coefficients giữa các frame (khi xe di chuyển, gốc tọa độ dịch)
  là vô nghĩa toán học — polynomial ở 2 thời điểm khác nhau không thể cộng tuyến tính
  thành polynomial hợp lệ. Xóa luôn `poly_ema_alpha` param từ cả 2 yaml + `perc_s_max`
  variable.

- [x] **Fix #4 — Startup kappa bias (mpc_node enabled_cb)**: Reset `kappa_smooth=0` khi
  `car_enabled` chuyển False→True. Trên map 1, khi xe đứng yên, perception publish
  cùng giá trị kappa=-0.1182 mỗi frame (static bias do sliding window không đối xứng tuyệt
  đối). Sau 4.5s chờ user nhấn start, `kappa_smooth` hội tụ về -0.1182. Khi start, MPC
  ngay lập tức dùng kappa=-0.12 → model nghĩ đường cong trái → steer gắt sang phải →
  feedback loop → oscillation. Fix: reset kappa về 0 ở startup, kappa thật build up từ
  perception data trong vài frame đầu chạy.

### Kappa Pipeline — Issues còn LẠI (chưa fix)

| # | Vấn đề | Mức | Ảnh hưởng |
|---|--------|-----|-----------|
| 5 | Double exponential weighting trong perception (u-space + s-space) | MAJOR | Near-field bias quá mạnh, far-field gần như bị bỏ |
| 6 | Sliding window drift ở upper windows | MODERATE | Tích lũy error window-over-window |
| 7 | Kappa noisy → EKF predict `alpha_dot` noisy → feedback loop | MODERATE | Amplify noise qua closed loop |

### Kappa Pipeline — Phương pháp thay thế polynomial (suggestions)

Polynomial fit có bias không tránh được (fit bậc 3 với 10 điểm luôn có curvature nhỏ
khác 0 dù đường thẳng). Các phương án thay thế có thể thử sau:

| Phương pháp | Ý tưởng | Ưu điểm | Nhược điểm |
|---|---|---|---|
| **Circle fitting** | Fit đường tròn qua center points → kappa=1/R | Kappa trực tiếp, robust, 1 giá trị | Chỉ 1 kappa, không model S-curve |
| **Menger curvature** | Kappa từ 3 điểm liên tiếp (tam giác ngoại tiếp) | Cực đơn giản, local | Nhạy noise nếu điểm gần nhau |
| **B-spline** | Thay polynomial bằng B-spline | Local control, ổn định hơn | Phức tạp hơn polyfit |
| **RANSAC + polyfit** | RANSAC loại outlier trước fit | Handle sliding window drift | Thêm latency |
| **Kalman filter (e_y, e_psi, kappa)** | KF tracking thay EMA | Model dynamics, tự adapt gain | Cần tune Q, R |

**Khuyến nghị:** Circle fitting hoặc Kalman filter nếu pipeline hiện tại vẫn còn vấn đề
trên hardware thật. Với sim hiện tại (sau 4 fixes trên), đủ mượt cho demo.

### Phase 1.5 Checklist (Hardware Deploy)

- [ ] Resolve Pi Camera 3 CFE driver issue on RPi OS
- [ ] Calibrate camera intrinsics (checkerboard)
- [ ] Recalibrate BEV homography SRC/DST for real camera → update `perception.yaml`
- [ ] Tune AdaptiveThreshold for black tape on white paper
- [ ] Write BNO055 serial parser node (Nucleo → `/dev/ttyACM0` → `/imu/data`)
- [ ] Test perception node on Pi, stream `/camera/image_raw` to laptop
- [ ] Deploy NMPC on laptop, receive `/perception/lane_state` from Pi over network
- [ ] Test at 0.3 m/s on straight first
- [ ] Record video milestone

---

## 9. KNOWN ISSUES & NOTES

### Code Issues Found

| Issue | Location | Detail |
|-------|----------|--------|
| `visualizer_node.py` hardcodes BEV SRC/DST | `visualizer_node.py` lines 17-18 | Does NOT read from `perception.yaml`. If SRC/DST changes, must update both files manually. |
| Spawn vs reset position mismatch (Map 1) | `sim_full_launch.py` line 18 vs `reset_node.py` line 17 | Spawn: x=1.5; Reset: x=0.0. Intentional (spawn at track entry, reset at lane center origin). |
| `visualizer_node.py` kappa formula differs from `perception_node.py` | `visualizer_node.py` line 137 | Visualizer uses `2*pc[1]*SCALE²` (approximate, pixel-space). perception_node uses correct arc-length formula `2b/(1+c²)^1.5`. Values may differ slightly. |
| `grid_viewer` launched by `sim_full_launch.py` | `sim_full_launch.py` line 117 | Requires display (GUI). Headless environments will fail. |
| `ekf_healthy: false` at startup | `ekf_node` | Run full pipeline (Gazebo + perception) before EKF to have sensor data. |

### Build Issues

| Issue | Solution |
|-------|----------|
| CMakeCache path issue | `rm -rf build/ install/` then rebuild |
| `build/lib/` stale cache for mpc_pkg | `rm -rf build/mpc_pkg install/mpc_pkg` then rebuild |
| lane_msgs build fail with symlink | `rm -rf build/lane_msgs install/lane_msgs` |
| NMPC solver not rebuilding (cached) | `rm -rf nmpc_build/` to force rebuild |

### Runtime Issues

| Issue | Root Cause | Solution |
|-------|-----------|---------|
| Gazebo segfault on load | Sensors plugin added twice | Do NOT add `ign-gazebo-sensors-system` to world SDF (already in xacro) |
| `No executable found` for ROS2 node | Empty `console_scripts` in setup.py | Add entry points to setup.py |
| IMU `/imu/data` no data | Missing `ignition-gazebo-imu-system` plugin | Already added to vehicle.xacro |
| NMPC warning `Gauss-Newton + EXTERNAL` | acados normal behavior | Safe to ignore |
| `reset_node` teleport code=255 "Invalid arguments" | Dùng `gz service` của Gazebo Classic thay vì `ign service` của Ignition Fortress | Fixed: dùng `ign service --reqtype ignition.msgs.Pose` |
| `reset_node` Q reset không teleport xe về vị trí ban đầu | (1) Protobuf format thiếu `:` trước nested message (`position { }` thay vì `position: {}`) — Fortress parser không chấp nhận; (2) Không có delay trước teleport → physics velocity còn → xe drift ngay sau teleport | Fixed: khôi phục format `position: {x:... y:... z:...}` + thêm delay 0.5s (5×0.1s publish stop) trước khi gọi `ign service` |
| Xe dao động lái trái-phải trên đường thẳng | raw perception `e_y`/`e_psi` inject trực tiếp vào NMPC initial state | Fixed: dùng EKF-filtered `n`/`alpha`; tăng `w_delta` 10→50→80 (`rm -rf nmpc_build/` cần thiết) |
| Xe văng khi vào cua DLC (map 3) | `fit_cubic_arclength` dùng toàn bộ sliding window points kể cả điểm ngoài `s_max_m` → far-field drift → polynomial bowing → kappa spike → `1-n*kappa_c` singularity trong MPC model | Fixed: clip `us_m ≤ s_max_m`, near-field exponential weighting, kappa từ quadratic fit, `kappa_max=3.0` clamp |
| Polynomial lines bowing outward (chữ V) trong LANES panel | Sliding window drift outward ở upper BEV windows (unreliable) kéo lệch cubic polynomial | Partially fixed: near-field weighting giảm impact; root cause vẫn là sliding window algorithm — acceptable cho sim, cần monitor trên hardware |
| `libEGL warning: egl: failed to create dri2 screen` | GPU driver | Safe to ignore |

### Hardware-Specific Notes

- EKF can be disabled for early hardware testing — NMPC falls back to first-order `v` model
- Pi Camera 3 Wide (IMX708): known issue "Unable to acquire a CFE instance" on Ubuntu Server 24.04 → switched to RPi OS 64-bit + Docker
- BNO055 on Nucleo F411RE: serial parser node not yet written (Phase 1.5 TODO)
- Docker volume on Pi: `~/ros2_ws:/ros2_ws`

---

## 10. NEXT STEPS (Phase 1.5)

### Immediate (Track 2 / Sim Validation)

```bash
# Test full pipeline on Map 2
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=2
# Then in reset_node terminal: s → start, observe tracking on chicane + S-curve
```

### Hardware Deploy Pipeline

```
Pi Camera (CSI)
  → picamera2 / ROS2 camera node on Pi
  → /camera/image_raw (Pi)
  → perception_node (Pi) — with recalibrated perception.yaml
  → /perception/lane_state (Pi → network → laptop)
  → mpc_node (laptop)
  → /steering_angle, /velocity (laptop → network → Pi)
  → Nucleo F411RE (Pi USB serial)
  → Servo (steering) + ESC (drive motor)
```

### BEV Recalibration Procedure (Hardware)

1. Print checkerboard, place flat on track at camera field of view
2. Run `camera_calibration` ROS2 package → get camera intrinsics
3. Identify 4 corners of lane marking rectangle in camera image → new `bev_src` values
4. Measure physical positions → compute `bev_dst` to match metric scale
5. Update `perception_pkg/config/perception.yaml` `bev_src` / `bev_dst`
6. Update `visualizer_node.py` lines 17-18 with same SRC values

### Phase 2 Planning (SLAM)

- Visual odometry: `image_proc` + optical flow
- Map format: custom lane graph (pickle/JSON)
- Localization: `robot_localization` package or particle filter on lane graph

### Phase 3 Planning (Nav2)

- Nav2 interface: `/cmd_vel` (linear.x, angular.z) → convert → NMPC reference
- Replace DWB with NMPC as local controller

# 01. Pipeline — Packages, Nodes, Topics, Messages

Updated 2026-04-19.

---

## Packages

| Package | Mô tả | Nodes |
|---------|-------|-------|
| `perception_pkg` | Camera → BEV → lane detection → `LaneState` | `perception_node`, `visualizer_node`, `grid_viewer` |
| `mpc_pkg` | NMPC 6-state (Kloeser 2020 extended) qua acados | `mpc_node` |
| `ekf_pkg` | EKF sensor fusion + reset / keyboard control | `ekf_node`, `reset_node` |
| `lane_msgs` | Custom ROS2 msgs | — (msgs only) |
| `gazebo_ackermann_steering_vehicle` | Sim world, URDF/xacro, C++ controller, launch | `vehicle_controller` (C++) |

---

## Node details

### `perception_node` (`perception_pkg/perception_node.py`)
- Sub: `/camera/image_raw`.
- Pub: `/perception/lane_state`.
- Reads params từ `perception.yaml`.

### `visualizer_node` (`perception_pkg/visualizer_node.py`)
- Sub: `/camera/image_raw`.
- Pub: `/perception/view1_raw`, `view2_bev`, `view3_threshold`,
  `view4_lanes`, `debug_grid`.
- ⚠️ BEV SRC/DST **hardcoded** trong visualizer (lines 17-18) — không đọc yaml,
  phải sync tay khi đổi `perception.yaml`.

### `grid_viewer` (`perception_pkg/grid_viewer.py`)
- Sub: `/perception/debug_grid`.
- Hiển thị 2×2 debug grid qua `cv2.imshow()` (GUI "Perception Debug [2x2]"
  ở 1280×1040). Fail trên môi trường headless.

### `mpc_node` (`mpc_pkg/mpc_node.py`)
- Sub: `/perception/lane_state`, `/ekf/vehicle_state`, `/car_enabled`.
- Pub: `/steering_angle`, `/velocity`.
- Reads params từ `nmpc.yaml` (map 3) hoặc `nmpc_map1.yaml`.
- Build acados solver vào `nmpc_build_6state/` (~5–30s lần đầu).

### `ekf_node` (`ekf_pkg/ekf_node.py`)
- Sub: `/imu/data`, `/perception/lane_state`, `/joint_states`.
- Pub: `/ekf/vehicle_state`.
- Nội bộ state: `[n, ṅ, α, α̇, v, ψ]`. Pub state dạng `VehicleState` msg.

### `reset_node` (`ekf_pkg/reset_node.py`)
- Sub: `/perception/lane_state`.
- Pub: `/velocity`, `/steering_angle`, `/car_enabled`.
- Service: `/reset_car` (std_srvs/Trigger).
- Keyboard (stdin TTY): `s`=START, `x`=STOP, `q`=RESET teleport.
- Auto-reset khi `lane_detected=False` > 3.0s (trong khi enabled).
- Param: `map_id` (int, default=1) — chọn spawn pose.

---

## Topics

| Topic | Type | Pub | Sub | Hz |
|-------|------|-----|-----|----|
| `/camera/image_raw` | sensor_msgs/Image | Gazebo bridge | perception, visualizer | 30 |
| `/imu/data` | sensor_msgs/Imu | Gazebo bridge | ekf | ~77 |
| `/joint_states` | sensor_msgs/JointState | Gazebo bridge | ekf | 100 |
| `/perception/lane_state` | lane_msgs/LaneState | perception | mpc, ekf, reset | 30 |
| `/perception/debug_grid` | sensor_msgs/Image | visualizer | grid_viewer | 30 |
| `/perception/view1_raw` .. `view4_lanes` | sensor_msgs/Image | visualizer | — | 30 |
| `/ekf/vehicle_state` | lane_msgs/VehicleState | ekf | mpc | 100 |
| `/car_enabled` | std_msgs/Bool | reset | mpc | 10 |
| `/steering_angle` | std_msgs/Float64 | mpc, reset | vehicle_controller | 20 |
| `/velocity` | std_msgs/Float64 | mpc, reset | vehicle_controller | 20 |

**Note:** `/steering_angle` và `/velocity` pub @ 20Hz = `ctrl_hz` của NMPC.

---

## Services

| Service | Type | Server | Mô tả |
|---------|------|--------|-------|
| `/reset_car` | std_srvs/Trigger | reset_node | Teleport xe về spawn, stop car |

---

## Custom messages

### `lane_msgs/LaneState` (perception @ 30 Hz)

```
std_msgs/Header header
float64 e_y        # lateral offset [m] tại s=0 (= coeff_d)
float64 e_psi      # heading error [rad] tại s=0 (= arctan(coeff_c))
float64 kappa      # curvature [1/m] tại s=0 (từ quadratic fit, đã smooth)
float64 coeff_a    # cubic: x_c(s) = a·s³ + b·s² + c·s + d
float64 coeff_b
float64 coeff_c
float64 coeff_d
float64 s_max      # camera lookahead [m]
bool    lane_detected
```

### `lane_msgs/VehicleState` (ekf @ 100 Hz)

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
bool    ekf_healthy
```

---

## Data flow (closed loop, sim)

```
Camera 30Hz → perception_node → /perception/lane_state
                                       │
IMU 77Hz ──→ ekf_node ──────→ /ekf/vehicle_state
Joint states 100Hz ┘                  │
                                       ↓
                              mpc_node (NMPC 6-state @ 20Hz)
                                   ↓              ↓
                          /steering_angle    /velocity
                                   ↓              ↓
                          vehicle_controller (C++, Gazebo plugin)
                                   ↓
                              Gazebo physics
```

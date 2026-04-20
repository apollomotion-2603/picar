# INDEX — Code Explanation

Tài liệu kỹ thuật chi tiết cho toàn bộ dự án **RC Car Autonomous Driving**.

Bản viết: 2026-04-20. Tác giả: Claude (dựa trên source code tại commit
đang active). Ngôn ngữ: tiếng Việt, thuật ngữ kỹ thuật giữ tiếng Anh.

> Đọc trước: [CLAUDE.md](../../../CLAUDE.md) cho project identity,
> roadmap, conventions.

---

## 1. PIPELINE TỔNG QUAN

```
       ┌──────────┐
Camera─┤ PiCam3   │
       └────┬─────┘ 30Hz /camera/image_raw
            │
      ┌─────▼───────┐
      │ perception  │─── /perception/lane_state (LaneState)
      │    node     │       │
      └─────┬───────┘       │
            │ (debug grid)  │
      ┌─────▼───────┐       │
      │ visualizer  │       │
      │    node     │       │
      └─────────────┘       │
                            │
                   ┌────────▼──────────┐
IMU /imu/data ────▶│    ekf_node       │─── /ekf/vehicle_state
joint_states  ────▶│ (fuse @100Hz)     │        (VehicleState)
/car_enabled  ────▶│                   │             │
                   └───────────────────┘             │
                                                     │
                   ┌─────────────────────────────────┘
                   ▼
            ┌──────────┐
/perception/lane_state ▶│  mpc_node  │─── /steering_angle
/car_enabled ──────────▶│ (NMPC @20Hz)│─── /velocity
                        └──────────┘         │
                                             ▼
                                      ┌─────────────┐
                                      │ vehicle_ctrl│  (Gazebo plugin)
                                      │  (Ackermann)│
                                      └─────────────┘
```

Control panel: `reset_node` (keyboard S/X/Q + service `/reset_car`).

---

## 2. MAP FILE → CHỨC NĂNG

| # | File | Node/Artifact | Mô tả |
|---|---|---|---|
| 01 | [01_perception_node.md](01_perception_node.md) | `perception_node.py` | BEV + sliding window + cubic arc-length fit → LaneState |
| 02 | [02_visualizer_node.md](02_visualizer_node.md) | `visualizer_node.py` | 4-panel debug grid 2×2 cho perception |
| 03 | [03_grid_viewer.md](03_grid_viewer.md) | `grid_viewer.py` | cv2.imshow() xem debug_grid (dev tool) |
| 04 | [04_ekf_node.md](04_ekf_node.md) | `ekf_node.py` | EKF 6-state fuse Camera/IMU/Encoder @100Hz |
| 05 | [05_reset_node.md](05_reset_node.md) | `reset_node.py` | START/STOP/RESET panel + auto-stop safety |
| 06 | [06_mpc_node_part1.md](06_mpc_node_part1.md) | `mpc_node.py` (init) | Params, solver build, sub/pub, enable edge |
| 07 | [07_mpc_node_part2.md](07_mpc_node_part2.md) | `mpc_node.py` (loop) | κ horizon, adaptive v_ref, solve + publish |
| 08 | [08_acados_settings.md](08_acados_settings.md) | `acados_settings.py` | OCP cost, constraints, solver options |
| 09 | [09_bicycle_model.md](09_bicycle_model.md) | `bicycle_model.py` | Dynamics CasADi 6-state Kloeser 2020 |
| 10 | [10_custom_messages.md](10_custom_messages.md) | `LaneState.msg`, `VehicleState.msg` | Field-by-field |
| 11 | [11_launch_and_world.md](11_launch_and_world.md) | `sim_full_launch.py`, `*.sdf`, `.xacro` | Launch orchestration + 4 map |

---

## 3. MAP PACKAGE → FILE

**`src/perception_pkg/`** (3 nodes):
- `perception_node.py` → [01](01_perception_node.md)
- `visualizer_node.py` → [02](02_visualizer_node.md)
- `grid_viewer.py` → [03](03_grid_viewer.md)
- `config/perception.yaml`, `perception_map1.yaml` → §5 file 01/02

**`src/ekf_pkg/`** (2 nodes):
- `ekf_node.py` → [04](04_ekf_node.md)
- `reset_node.py` → [05](05_reset_node.md)

**`src/mpc_pkg/`** (1 node + 2 lib):
- `mpc_node.py` → [06](06_mpc_node_part1.md) + [07](07_mpc_node_part2.md)
- `acados_settings.py` → [08](08_acados_settings.md)
- `bicycle_model.py` → [09](09_bicycle_model.md)
- `config/nmpc.yaml`, `nmpc_map1.yaml` → §5 file 06

**`src/lane_msgs/`** (msg package):
- `LaneState.msg`, `VehicleState.msg` → [10](10_custom_messages.md)

**`src/gazebo_ackermann_steering_vehicle/`** (sim infra):
- `launch/sim_full_launch.py`, `vehicle.launch.py`, `track_test_launch.py`
  → [11](11_launch_and_world.md)
- `model/vehicle.xacro` → §5 file 11
- `worlds/*.sdf` (4 tracks) → §6 file 11

---

## 4. KÝ HIỆU TOÁN HỌC THỐNG NHẤT

| Ký hiệu | Ý nghĩa | Topic/State |
|---|---|---|
| `s` | arc-length | NMPC state[0] |
| `n` ≡ `e_y` | lateral offset [m] | LaneState.e_y, VehicleState.n, NMPC state[1] |
| `α` ≡ `e_psi` | heading error [rad] | LaneState.e_psi, VehicleState.alpha, NMPC state[2] |
| `v` | speed [m/s] | VehicleState.v, NMPC state[3] |
| `D` | throttle [-1,1] | NMPC state[4] |
| `δ` | steering [rad] | NMPC state[5], /steering_angle |
| `κ` | curvature [1/m] | LaneState.kappa, NMPC param `p` |
| `ψ` | yaw abs [rad] | VehicleState.psi |

Vehicle consts: `m=1.5kg sim / 2.5kg HW`, `lf=lr=0.11m`, `L=0.22m`,
`wheel_r=0.04m`, `δ_max=0.6109rad≈35°`, `lane_width=0.5m`.

---

## 5. CÁC CROSS-CUTTING CONCERN

### 5.1 "Khi nào cần `rm -rf nmpc_build_6state/`?"
- Sửa `bicycle_model.py` (dynamics/constraint expr).
- Sửa `acados_settings.py` (cost W/W_e, constraint bounds,
  `idxsh/idxbu/idxbx`, solver options, Tf/N).
- KHÔNG cần: sửa `target_v, kappa_max, kappa_ema_alpha` (runtime params Python).

### 5.2 "Param ở đâu?"
- Hardcoded Python: EKF hằng số (WHEEL_RADIUS, noise matrices),
  reset_node timeouts, visualizer layout.
- YAML: perception.yaml, nmpc.yaml (+ map1 variants), parameters.yaml (xe).
- XACRO args: kích thước xe.
- SDF: spawn pose, ground texture, physics step.

### 5.3 "Topic/QoS"
Chủ yếu default RELIABLE depth 10. Visualizer dùng BEST_EFFORT depth 1 cho
ảnh để giảm lag.

### 5.4 "Map-specific tuning"
Map 1/2/4 (oval, chicane) → `*_map1.yaml` (narrower win, higher target_v
khả dĩ).
Map 3 (DLC) → `nmpc.yaml` + `perception.yaml` (wider win, lower target_v,
stronger kappa smoothing).

---

## 6. DEBUG CHEAT SHEET

| Triệu chứng | Chỗ check |
|---|---|
| MPC publish v=0 liên tục | `lane_detected`? `car_enabled`? EKF healthy? |
| Solver status != 0 | `kappa_max` quá nhỏ? `alat_max` quá chặt? Log t_ms |
| Xe lắc khi cua | `kappa_ema_alpha` tăng? Q[n] lớn quá? |
| EKF `n` drift | IMU timeout? `lane_cb` chạy không? std_n tăng |
| Tune yaml không hiệu quả | Quên rebuild? Chạy trên cache cũ? map_id đúng? |
| Auto-stop giữa DLC | `N_MAX_STOP` trong reset_node.py |

Log format NMPC (CLAUDE.md §5):
```
n=Xmm α=X° v=X(src) D=X δ=X° κ₀=X vref₀=X v→X t=Xms st=X
```

---

## 7. LIMITATIONS & TECH DEBT

- Hardcoded workspace path `~/main/1_projects/…` ở:
  - `mpc_node.py` dòng 116.
  - `sim_full_launch.py` dòng 48.
  - `lane_track.sdf` albedo_map.
- `MAP_CONFIGS` duplicate (launch + reset_node).
- EKF hardcoded vehicle params, không load yaml.
- Visualizer default BEV corners khác yaml → phải luôn `--params-file`.
- Visualizer sliding_window nhân bản từ perception_node.
- `ekf_alpha` clip bằng `DELTA_MAX` (physical limit steering), không đúng
  ý nghĩa heading error.
- Dead code visualizer line 192–193.
- Thesis writing vẫn pending (deadline 2026-04-25, còn 5 ngày).

# 05. Status, Completed Work, Known Issues

Updated 2026-04-19.

---

## Phase progress

| Phase | Tuần | Mô tả | Status |
|-------|------|-------|--------|
| 1 | 1–4 | Simulation core (perception + NMPC + EKF) | ✅ DONE |
| 1.5a | 5–6 | Sim tracks (map 1 + 3 đủ dùng; map 2, 4 drop scope) | ✅ DONE |
| 1.5b | 7 | Hardware lane following 0.3 m/s | 🔄 NOW |
| — | — | Thesis writing (chương 2-5) | 🔄 song song |
| 2 | 8 | SLAM (VO + lane graph + PF) | 📅 scope giảm |
| 3 | 9–10 | Navigation (Nav2 + obstacle) | 📅 scope giảm |
| 4 | 11 | GUI Foxglove | 📅 cắt nếu thiếu giờ |
| 5 | 12 | Luận văn + demo | 🔄 viết song song |

---

## Completed (Phase 1)

### Simulation core
- [x] Ignition Gazebo Fortress sim (4 world SDF).
- [x] Camera BEV + lane detection + cubic arc-length polynomial.
- [x] Sliding window với adaptive threshold.
- [x] EKF fusion: IMU predict + camera update + joint_states v.
- [x] **NMPC 6-state** (Kloeser 2020 extended) — acados SQP_RTI + HPIPM,
  LINEAR_LS cost, runtime κ param.
- [x] Curvature-based adaptive v_ref per-stage trong cost.
- [x] EKF `v/n/α` feedback vào NMPC initial state.
- [x] Reset node: keyboard + service + auto-stop lane lost 3s + Ignition teleport.
- [x] `sim_full_launch.py` — 1 lệnh, reset_node chạy riêng T2.
- [x] 4-panel debug visualizer (raw, BEV, threshold, lanes).

### Sim smoothing
- [x] Joint damping giảm: wheels 0.001, steering 0.01.
- [x] `body_density` 7850 → 925 (ABS) → mass ≈ 2.5 kg đúng.
- [x] Rear wheel mass: density formula (~0.09 kg/wheel).
- [x] EKF-filtered `n/α/v` thay raw perception cho NMPC init.
- [x] Reset teleport: `ign service` + `ignition.msgs.Pose` + protobuf `:` format
  + delay 0.5s trước teleport để physics settle.

### Kappa pipeline overhaul (Apr 2026)
- [x] Fix #1 — `sliding_window()` trả `center_pts` chỉ từ windows có CẢ HAI
  lane cùng Y (bỏ zip-by-index buggy).
- [x] Fix #2 — Horizon κ nhất quán: dùng `kappa_smooth` scalar (khi rollback)
  hoặc từ cubic + EMA (hiện tại, broken — đang debug).
- [x] Fix #3 — Xóa EMA trên polynomial coefficients `poly_a/b/c/d` — trộn
  coeffs giữa frames là vô nghĩa toán học (gốc tọa độ dịch khi xe chạy).
- [x] Fix #4 — Reset `kappa_smooth=0` khi `car_enabled` False→True — tránh
  κ bias tích lũy khi xe đứng yên (map 1 bias -0.12).
- [x] Polynomial bowing fix: clip `us_m ≤ s_max_m`, near-field exp weighting,
  κ từ quadratic fit.
- [x] `kappa_max` per-map: 2.0 (map1) / 0.8 (map3).

### NMPC velocity logic (Hướng A)
- [x] Bỏ `_adaptive_velocity()` override tại publish.
- [x] Adaptive `v_ref` nằm trong cost reference per-stage.
- [x] Publish `v_cmd = clip(x1[3], 0, v_max)` — trust solver.
- [x] Thêm log `vref₀`.

---

## Active issues

_Không còn active issue. Sim closed — xem mục "Sim closed" dưới._

---

## Sim closed (2026-04-19)

### ✅ Map 3 DLC dao động — RESOLVED

**Timeline:**
- Bag 7 diagnosed: `|n|max=97mm`, `|κ|max=0.97`, δ flip rate cao.
- Bag 8 regression với per-stage EMA (v-dependent) — `δ` saturate ±35°,
  `v` stall. Root cause: `s_j = v·j·dt` → stage j giữa frames khác nhau
  về arc-length vật lý, EMA vô nghĩa.
- Bag 10 scalar EMA broadcast constant: stable nhưng **mất preview** →
  xe lệch trái sớm đầu cua (`ey` mean -14mm).
- **Bag 11 final fix:** per-stage EMA với `s_j = j·s_max/N` (arc-length
  uniform, v-independent). Giữ preview 1s **và** EMA hợp lệ.
  - `|n|max=72mm` · `|κ|max=0.76` · δmax=14.8° · không saturate/stall.
  - Early `ey=+1.5±3.7mm` — bias cua đầu fix hoàn toàn.

**Remaining minor:** late-steer ~0.3s ở đoạn cuối DLC (EMA α=0.7 lag).
Chấp nhận — trên HW chạy 0.3 m/s (3× chậm hơn sim) lag này biến mất.

**Code:** `mpc_node.py:_compute_kappa_horizon()` — per-stage arc-length EMA.
**Config:** `nmpc.yaml` giữ `kappa_max=0.8`, `kappa_ema_alpha=0.7`.

---

## Known issues (code)

| Issue | Location | Detail |
|-------|----------|--------|
| `visualizer_node.py` hardcode BEV | lines 17-18 | KHÔNG đọc yaml. Sync tay khi đổi perception.yaml. |
| Spawn ≠ reset Map 1 | `sim_full_launch.py:18` vs `reset_node.py:17` | Intentional — spawn track entry, reset lane origin. |
| κ formula visualizer khác perception | `visualizer_node.py:137` | Visualizer `2·pc[1]·SCALE²` (pixel approx). Perception `2b_quad/(1+c²)^1.5`. Lệch nhẹ. |
| `grid_viewer` needs GUI | `sim_full_launch.py:117` | Headless env fail. |
| `ekf_healthy=false` khi startup | `ekf_node` | Cần Gazebo + perception chạy trước mới có sensor data. |
| Per-stage κ EMA | `mpc_node.py:224-261` | ✅ Fixed 2026-04-19 — arc-length uniform `s_j=j·s_max/N`. |

---

## Build issues

| Issue | Solution |
|-------|----------|
| CMakeCache path conflict | `rm -rf build/ install/` rebuild |
| `build/lib/` stale cache mpc_pkg | `rm -rf build/mpc_pkg install/mpc_pkg` |
| lane_msgs build fail với symlink | `rm -rf build/lane_msgs install/lane_msgs` |
| NMPC solver stale (yaml/model changed) | `rm -rf nmpc_build_6state/` |

---

## Runtime issues

| Issue | Root cause | Solution |
|-------|-----------|---------|
| Gazebo segfault load | Sensors plugin add twice | KHÔNG add `ign-gazebo-sensors-system` vào world SDF |
| `No executable found` | Empty `console_scripts` | Thêm entry_points vào setup.py |
| `/imu/data` không data | Missing `ignition-gazebo-imu-system` plugin | Đã add vào `vehicle.xacro` |
| NMPC warn *Gauss-Newton + EXTERNAL* | Acados normal | Bỏ qua |
| `reset_node` teleport code 255 | Dùng `gz service` Gazebo Classic | Fix: `ign service` với `ignition.msgs.Pose` |
| `reset_node` Q không teleport | Protobuf thiếu `:` trước nested + physics velocity chưa dừng | Fix: format `position: {x:...}` + delay 0.5s |
| Xe dao động đường thẳng | Raw perception inject NMPC init | Fix: EKF-filtered n/α; tuning weight stage cost |
| Xe văng DLC map 3 cua | Polynomial bowing + κ spike | Fix: near-field weighting, κ quadratic, `kappa_max=0.8` |
| LANES panel chữ V outward | Sliding window drift upper windows | Partially fixed: near-field weight; root cause còn |
| `libEGL warning: egl: failed to create dri2 screen` | GPU driver | Bỏ qua |

---

## Hardware-specific (planned — chưa test)

- EKF có thể disable giai đoạn đầu — NMPC fallback first-order v.
- Pi Camera 3 Wide (IMX708): "Unable to acquire CFE instance" trên Ubuntu
  Server 24.04 → switched RPi OS 64-bit + Docker.
- BNO055 serial parser: chưa viết (Phase 1.5b TODO).
- Docker volume Pi: `~/ros2_ws:/ros2_ws`.

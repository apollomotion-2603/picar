# 10 — Custom Messages: LaneState & VehicleState

Package: `src/lane_msgs/`
- `msg/LaneState.msg` (24 dòng)
- `msg/VehicleState.msg` (11 dòng)
- `CMakeLists.txt` + `package.xml`

---

## 1. MỤC ĐÍCH

2 message custom liên kết 3 node chính:

```
perception_node ──LaneState──> ekf_node ──VehicleState──> mpc_node
                    │                                       ▲
                    └───────────────(bypass)────────────────┘
```

- `LaneState` (@30 Hz): output của perception, input của EKF và MPC.
- `VehicleState` (@100 Hz): output của EKF, input chính của MPC.

---

## 2. CẤU TRÚC PACKAGE

`CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneState.msg"
  "msg/VehicleState.msg"
  DEPENDENCIES std_msgs
)
```
Chuẩn ROS2 — tạo Python/C++ binding từ `.msg` thông qua `rosidl`. Khi
thêm field hoặc message mới → rebuild `colcon build --packages-select
lane_msgs` + resource downstream.

---

## 3. LaneState.msg — FIELD BY FIELD

```
std_msgs/Header header

# Lateral offset from lane center (n in NMPC state) [m]
float64 e_y

# Heading error relative to lane tangent (alpha in NMPC state) [rad]
float64 e_psi

# Curvature of centerline at s=0 [1/m]
float64 kappa

# Cubic polynomial centerline in BEV arc-length frame:
#   x_c(s) = coeff_a*s^3 + coeff_b*s^2 + coeff_c*s + coeff_d
float64 coeff_a
float64 coeff_b
float64 coeff_c
float64 coeff_d

# Maximum valid arc-length (camera lookahead limit) [m]
float64 s_max

# Whether lane was successfully detected
bool lane_detected
```

### Giải thích field

- **`header`**: chuẩn ROS2, `stamp` gán `now()` tại perception (camera
  frame time), `frame_id = 'camera'`. EKF/MPC không hiện dùng timestamp
  cho interpolation (trust realtime), nhưng có sẵn cho bag analysis.

- **`e_y`** (m): lateral offset của xe so với centerline. Dấu: âm khi xe
  lệch trái, dương lệch phải (theo BEV convention, +x phải). Trùng với
  state `n` trong NMPC, MPC pass vào `x0[1]`.
  - Range thực tế: ±0.25 m (lane width 0.5 m). MPC clip `±n_max=0.20`.

- **`e_psi`** (rad): heading error. Dấu: dương khi xe quay phải so với
  tangent. Tính từ cubic: `e_psi = arctan(coeff_c)`. Trùng với `α`.
  - Range: ±0.6 rad (±35°) sau khi perception_node detect thành công.

- **`kappa`** (1/m): curvature tại `s=0` (vị trí xe). Tính từ **quadratic
  fit** (không phải cubic!) để mượt hơn. Dấu: dương khi cua phải.
  - Range sim: ±0.8 (sau clamp KAPPA_MAX).
  - Notice: EKF dùng qua EMA (α=0.7), MPC KHÔNG dùng trực tiếp field này
    mà tính lại từ `coeff_*` per-stage.

- **`coeff_a, coeff_b, coeff_c, coeff_d`** (m): hệ số cubic
  `x_c(s) = a·s³ + b·s² + c·s + d`. Đây là centerline tại toạ độ xe.
  - `coeff_d` = `x_c(0)` — same sign & magnitude với `e_y`.
  - `coeff_c` = `x_c'(0)` — tương đương `tan(e_psi)`.
  - `coeff_b`, `coeff_a` dùng cho MPC tính κ per-stage (xem file 07).

- **`s_max`** (m): arc-length xa nhất có data. Tùy thuộc track + camera.
  - Thực tế sim: 0.5–1.0 m (s_max_m config 0.7, nhưng fit có thể ngắn hơn).
  - MPC dùng `s_max` làm horizon target `s_horizon = linspace(0, s_max, N+1)`.

- **`lane_detected`** (bool): `False` nếu `mode == LOST` hoặc fit fail.
  - Downstream behavior:
    - EKF: skip update (vẫn predict).
    - MPC: fail-safe stop (`v=0, δ=0`).
    - reset_node: nếu true 1.5 s liên tục → auto-stop.

### Gotchas

- `coeff_a..d` order theo numpy `np.polyfit` convention — **highest first**.
  Nếu viết handler khác (eg C++), phải nhớ `[a_cubic, b_quad, c_lin, d_const]`.
- `kappa` (field) từ quadratic ≠ `κ(s=0)` tính từ `coeff_*` cubic. Sai
  khác nhỏ nhưng tồn tại. File 01 giải thích lý do.
- `s_max` có thể = 0 khi fit fail → MPC fallback `0.5`. Điều kiện
  safety: `s_max > 0.05`.

---

## 4. VehicleState.msg — FIELD BY FIELD

```
std_msgs/Header header
float64 n
float64 n_dot
float64 alpha
float64 alpha_dot
float64 v
float64 psi
float64 std_n
float64 std_v
float64 std_alpha
bool ekf_healthy
```

### Giải thích field

- **`header`**: `stamp = now()` tại EKF publish (100 Hz), `frame_id = 'body_link'`.

- **`n`** (m): lateral offset — EKF estimate của `e_y`, chính xác hơn (fuse
  IMU + velocity).
- **`n_dot`** (m/s): vận tốc lateral.
- **`alpha`** (rad): heading error — EKF estimate của `e_psi`.
- **`alpha_dot`** (rad/s): heading rate. Tính trong predict: `yaw_rate − κ·ṡ`.
- **`v`** (m/s): speed dọc body x-axis. Seed 0.1, clip `[0, 2.0]`.
- **`psi`** (rad): yaw tuyệt đối. **Chỉ integrate yaw_rate**, drift theo
  thời gian. Không dùng bởi MPC (spatial model) — dự phòng cho SLAM phase 2.

- **`std_n, std_v, std_alpha`**: sqrt(diag(P)) — độ lệch chuẩn post-update.
  - Đơn vị khớp với state tương ứng.
  - MPC hiện chưa dùng, nhưng sẵn cho adaptive gain (giảm tin state khi
    std cao).

- **`ekf_healthy`** (bool): `True` khi mọi `diag(P) ∈ (0, 100)`. MPC check
  field này → nếu `False` fallback dùng raw `LaneState.e_y, e_psi`.

### Gotchas

- **Thiếu `D, delta`**: 2 phần tử cuối của NMPC state (throttle + steering)
  KHÔNG có trong VehicleState. EKF không estimate chúng — MPC warm-start
  từ previous prediction (`self.current_state[4], [5]`). Điều này OK vì
  throttle/steering là actuator command, không cần observer.
- **Không publish P full** — chỉ diag. Off-diagonal (correlation) mất. Đủ
  cho monitoring, thiếu cho advanced filter downstream.
- **`std_alpha` order**: khai báo sau `std_v`, khớp với gán
  `msg.std_alpha = sqrt(P[2,2])`. Đọc sai order → debug khó.
- **`psi` drift**: không có magnetometer / VIO, chỉ integrate yaw_rate →
  `psi` drift mỗi minute tùy IMU bias. Không critical cho lane following.

---

## 5. DEPENDENCIES

`package.xml` (implicit):
- `std_msgs` cho Header.
- `rosidl_default_generators` (build) + `rosidl_default_runtime` (exec).

Downstream consumers:
- `perception_pkg/perception_node.py`
- `perception_pkg/visualizer_node.py` (sub LaneState for overlay)
- `ekf_pkg/ekf_node.py`
- `ekf_pkg/reset_node.py` (sub LaneState for auto-stop)
- `mpc_pkg/mpc_node.py`

---

## 6. DESIGN RATIONALE

**Tại sao dùng custom msg thay vì nav_msgs/Path + Float64?**
- `LaneState` gộp 9 field dùng đồng thời trong 1 topic → sync guarantee
  (tất cả field cùng timestamp). Nếu publish 4 topic riêng → race
  condition giữa kappa, coeffs, e_y.
- `VehicleState` tương tự — 10 field EKF.
- Semantic rõ ràng: tên field khớp ký hiệu toán học paper Kloeser.

**Tại sao không thêm `e_y_ekf` vào LaneState?**
- Separation of concerns: LaneState thuần raw perception, VehicleState
  là filtered state. MPC fusion logic (prefer EKF if healthy) nằm ở
  consumer.

---

## 7. GOTCHAS TỔNG

- Đổi field → mọi node consume phải rebuild binding. Quên → `AttributeError`
  runtime khó debug.
- Field order trong `.msg` không quan trọng với Python/C++ (access by
  name), nhưng khi đọc ROS bag bằng CLI (`ros2 topic echo`) dễ đọc theo
  thứ tự khai báo.
- Thiếu units trong field names (`e_y` vs `e_y_m`) — comment ở `.msg` bù
  lại, nhưng dev mới dễ nhầm.
- `bool` default `False` khi msg khởi tạo → perception_node phải explicit
  set `lane_detected=True` khi fit OK.

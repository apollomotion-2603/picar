# 06 — mpc_node.py (Part 1: init, params, subs/pubs)

File: `src/mpc_pkg/mpc_pkg/mpc_node.py` dòng 1–225 (phần đầu 407-dòng node).
Part 2 tiếp tục giải thích horizon + solve loop.

---

## 1. MỤC ĐÍCH

NMPC 6-state lane following theo paper Kloeser 2020 (singularity-free
spatial bicycle). Subscribe perception + EKF → solve OCP bằng
**acados RTI + HPIPM** → publish `/steering_angle`, `/velocity` @ 20 Hz.

- Input: `/perception/lane_state` (30 Hz), `/ekf/vehicle_state` (100 Hz),
  `/car_enabled` (10 Hz).
- Output: `/steering_angle` (Float64), `/velocity` (Float64).
- Tần suất điều khiển: `ctrl_hz = 20 Hz` (timer 50 ms).

---

## 2. CẤU TRÚC FILE

Class `NMPCNode(Node)`:
- `__init__`: declare ~30 param, load, build solver, warm-start, pub/sub.
- `lane_cb`, `ekf_cb`, `enabled_cb`: sensor/status callbacks.
- `_compute_kappa_horizon`: per-stage κ(s_j) từ cubic + EMA.
- `_compute_v_ref_horizon`: adaptive v_ref từ κ.
- `control_cb`: timer 20 Hz — build x0, set yref, solve, publish.
- `_warm_start`, `_publish`: helpers.

---

## 3. GIẢI THÍCH KHỐI CODE — PHẦN KHỞI TẠO

### 3.1 State vector & control

Comment top-file:
```
State:   x = [s, n, alpha, v, D, delta]
Control: u = [derD, derDelta]
```
- `s` arc-length, `n` lateral offset, `α` heading error (giống EKF).
- `v` speed, `D` throttle normalized [−1, 1], `δ` steering rad.
- Control là **derivative** `[Ḋ, δ̇]` — tăng smooth, chống bang-bang.

### 3.2 Declare params

~30 params chia nhóm:
- **Vehicle geometry**: `lf=lr=0.11` (m).
- **Traction model**: `mass=1.5`, `C1=0.5`, `C2=15.5`, `Cm1=0.28`,
  `Cm2=0.05`, `Cr0=0.006`, `Cr2=0.011`, `cr3=5.0` — hệ số force/drag,
  giải thích ở file 09 bicycle_model.
- **Solver**: `N=20`, `Tf=1.0 s`, `ctrl_hz=20`, `timeout=1.0`.
- **Bounds**: `delta_max=0.6109 rad (≈35°)`, `n_max=0.20`,
  `throttle ±1`, `ddelta_max=2 rad/s`, `dthrottle_max=10 /s`,
  `v ∈ [0, 1.5] m/s`, `alat_max=along_max=4 m/s²`.
- **κ safety**: `kappa_max=0.8` (clip κ trước đưa vào solver).
- **Speed adaptation**: `kappa_speed_factor=2.0`, `v_min_curve=0.3`.
- **κ EMA**: `kappa_ema_alpha=0.7`.
- **EKF**: `ekf_timeout=1.0`.
- **Solver build**: `solver_build_dir='nmpc_build_6state'`.
- **Reference**: `target_v=1.0`.

**Gotcha**: default trong code KHÁC yaml. VD `target_v` default 0.6 nhưng
yaml 1.0. Luôn launch với `--params-file config/nmpc.yaml`.

### 3.3 Load params vào attribute

Chuẩn ROS2: `self.get_parameter(name).value`. Convention viết HOA:
`self.N_MAX`, `self.DELTA_MAX`, …. Giúp phân biệt hằng số config với state.

### 3.4 Build acados solver

```python
ws = os.path.expanduser(
    '~/main/1_projects/1_autonomous_car_research/ros2_ws')
build_dir = os.path.join(ws, solver_build_dir)

self.constraint, self.model_ns, self.solver = acados_settings(
    self.TF, self.N, build_dir,
    m=self.MASS, C1=self.C1, C2=self.C2,
    Cm1=self.CM1, Cm2=self.CM2,
    Cr0=self.CR0, Cr2=self.CR2, cr3=self.CR3,
    n_max=self.N_MAX, delta_max=self.DELTA_MAX,
    throttle_min=self.THROT_MIN, throttle_max=self.THROT_MAX,
    ddelta_max=self.DDELTA_MAX, dthrottle_max=self.DTHROT_MAX,
    alat_max=self.ALAT_MAX, along_max=self.ALONG_MAX,
)
```

- `acados_settings` (file 08) build OCP và return `(constraint_struct,
  model_namespace_str, AcadosOcpSolver_instance)`.
- `build_dir` là path tuyệt đối; acados cache generated C code tại đây.
  **Khi đổi params model/bounds/horizon** phải `rm -rf build_dir` nếu không
  sẽ dùng cache cũ → bug im lặng (xem CLAUDE.md §5).
- `ws` **hardcoded path** `~/main/1_projects/…/ros2_ws` → node **không
  portable**. TODO: lấy từ `ament_index_python` hoặc env `ROS_WORKSPACE`.

### 3.5 State attributes

```python
self.current_state = np.zeros(6)

self.kappa_horizon_smooth = None   # EMA per-stage κ
self.lane_coeffs  = None   # [a, b, c, d] cubic
self.lane_s_max   = 0.5
self.kappa_raw    = 0.0
self.lane_ok      = False
self.last_lane_time = self.get_clock().now()

# Fallback khi EKF chưa healthy
self.perc_n     = 0.0
self.perc_alpha = 0.0

# EKF cache
self.ekf_v = self.ekf_n = self.ekf_alpha = None
self.ekf_healthy = False

self.car_enabled = False
```

- `current_state` dùng cho warm-start giữa các solve — giữ prediction
  stage-1 frame trước.
- `kappa_horizon_smooth` là vector `(N+1,)` EMA, đồng bộ frame-to-frame vì
  `s_j = j · s_max/N` (arc-length uniform, không phụ thuộc `v`).

### 3.6 `_warm_start()`

```python
x0 = self.current_state.copy()
for i in range(self.N + 1):
    self.solver.set(i, 'x', x0)
    self.solver.set(i, 'p', np.array([0.0]))
for i in range(self.N):
    self.solver.set(i, 'u', np.array([0.0, 0.0]))
```
Set toàn bộ stage về `current_state` và `u=0`, param `κ=0`. Gọi khi start
hoặc khi enable False→True.

### 3.7 Pub/Sub

```python
self.pub_delta = self.create_publisher(Float64, '/steering_angle', 10)
self.pub_vel   = self.create_publisher(Float64, '/velocity', 10)

self.create_subscription(LaneState,    '/perception/lane_state', ..., 10)
self.create_subscription(VehicleState, '/ekf/vehicle_state',     ..., 10)
self.create_subscription(Bool,         '/car_enabled',           ..., 10)

self.timer = self.create_timer(1.0 / self.CTRL_HZ, self.control_cb)
```
QoS default (RELIABLE, depth 10). Timer 20 Hz = 50 ms period.

### 3.8 Callbacks sensor

**`lane_cb(msg)`**: chỉ update khi `msg.lane_detected`.
- Cache `perc_n = e_y`, `perc_alpha = e_psi`, `kappa_raw = kappa`.
- Lưu `lane_coeffs = [a, b, c, d]`, `lane_s_max = s_max`.
- Update `last_lane_time` (dùng cho timeout 1 s).

**`ekf_cb(msg)`**: chỉ cache khi `ekf_healthy=True`. Nếu unhealthy set flag
false (không xoá giá trị cũ — để fallback mượt).

**`enabled_cb(msg)`**: detect edge False→True:
```python
if not prev and self.car_enabled:
    self.current_state = np.zeros(6)
    if self.ekf_healthy and self.ekf_v is not None:
        self.current_state[3] = self.ekf_v
    self.kappa_raw = 0.0
    self.lane_coeffs = None
    self.kappa_horizon_smooth = None
    self._warm_start()
```
- Reset 6-state, seed `v` từ EKF nếu có (xe có thể đang lăn khi start).
- Clear EMA smooth → frame đầu dùng raw κ.
- Warm-start lại solver.

---

## 4. TOÁN HỌC (INIT PHASE)

OCP signature (full ở file 08):
```
min   Σ_{j=0}^{N-1} |y_j − y_ref_j|²_Q  +  |y_N − y_ref_N|²_Q_N
s.t.  x_{j+1} = f(x_j, u_j, κ_j)         ← RK4 bicycle model
      x_0 fixed (EKF state)
      constraints: |n| ≤ n_max, |δ| ≤ δ_max, throttle ∈ [-1, 1],
                   |u| ≤ max rate, nonlinear: |a_lat|, |a_long| ≤ limit
```

---

## 5. THAM SỐ YAML KEY

| Param | Default yaml (nmpc.yaml) | Ý nghĩa |
|---|---|---|
| `N` | 20 | Số horizon step |
| `Tf` | 1.0 s | Horizon time → `dt = 50 ms` |
| `ctrl_hz` | 20 | Control freq |
| `kappa_max` | 0.8 | Clip κ chống outlier |
| `kappa_ema_alpha` | 0.7 | EMA smoothing 0.7 old + 0.3 new |
| `kappa_speed_factor` | 2.0 | `v_ref = target_v/(1+factor·|κ|)` |
| `v_min_curve` | 0.3 | Floor cho v_ref khi cua |
| `target_v` | 1.0 | v reference |
| `delta_max` | 0.6109 | ≈ 35° |
| `n_max` | 0.20 | Lane hard-bound |
| `solver_build_dir` | nmpc_build_6state | acados cache |

Map-1 yaml (`nmpc_map1.yaml`) khác ở `target_v`, `kappa_*`, `v_min_curve`
— xem file riêng.

---

## 6. GOTCHAS (Part 1)

- **Hardcoded workspace path** dòng 116–117 — không portable.
- **Default params ≠ yaml** — luôn launch với params-file.
- **Quên `rm -rf nmpc_build_6state/`** khi đổi model/bounds → solver dùng
  cache cũ, hiện tượng: log nói "NMPC running" nhưng output khác sau khi
  sửa yaml. Ghi rõ trong CLAUDE.md §5.
- **Biến `self.current_state[0] = s`** reset 0 mỗi frame (xem Part 2) — s
  global không có ý nghĩa trong context lane-following spatial.
- **EMA key insight**: `s_j = j · s_max/N` arc-length uniform → stage j
  ổn định giữa các frame nên EMA per-stage hợp lệ. Nếu chuyển về
  `s_j = v·j·dt` (time uniform) thì EMA sai — từng frame stage j tương
  ứng điểm vật lý khác nhau.
- **`self.ekf_alpha` clip `[-DELTA_MAX, DELTA_MAX]`** (dòng 293) — nhầm lẫn
  ý nghĩa: α là heading error (có thể lớn), δ là steering angle (bound
  vật lý). Trong code clip α bằng `DELTA_MAX` để ép vào feasible region của
  solver, không cứng theo toán. Nếu cua lớn bị saturate.

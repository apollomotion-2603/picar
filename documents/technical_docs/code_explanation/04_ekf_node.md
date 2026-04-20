# 04 — ekf_node.py

File: `src/ekf_pkg/ekf_pkg/ekf_node.py` (266 dòng)

---

## 1. MỤC ĐÍCH

Fuse 3 nguồn sensor không đồng bộ:
- Camera 30 Hz → `LaneState` (e_y, e_psi, kappa).
- IMU 100 Hz → `Imu` (ax, ay, yaw_rate).
- Wheel encoder từ `JointState` 100 Hz → velocity.

Output: `VehicleState` @ 100 Hz gồm `[n, n_dot, alpha, alpha_dot, v, psi]`
cộng với độ bất định (std) và flag healthy. MPC dùng trực tiếp state này.

Tần suất: timer 100 Hz cho predict; update theo callback từng sensor.

---

## 2. CẤU TRÚC FILE

- Hằng số vehicle: `WHEEL_RADIUS=0.04`, `LF=LR=0.11`, `L=0.22`.
- Class `RCCarEKF`: filter thuần numpy (không ROS).
  - `__init__`: x, P, Q, R matrices, H matrices.
  - `predict(ax, ay, yaw_rate, kappa, dt)`.
  - `update(z, H, R)`, `update_camera`, `update_camera_and_velocity`.
  - `is_healthy`, `reset`.
- Class `EKFNode(Node)`: adapter ROS2.
  - Callbacks: `imu_cb`, `lane_cb`, `joint_cb`, `enabled_cb`.
  - Timer: `ekf_cb` @ 100 Hz.

---

## 3. GIẢI THÍCH KHỐI CODE

### 3.1 State vector

```python
# State: x = [n, n_dot, alpha, alpha_dot, v, psi]
self.x = np.zeros(6)
self.x[4] = 0.1   # v initial
```

- `n` — lateral offset so với centerline (m).
- `n_dot` — vận tốc lateral (m/s).
- `alpha` — heading error so với tangent centerline (rad).
- `alpha_dot` — vận tốc heading error (rad/s).
- `v` — speed dọc theo body x-axis (m/s), seed 0.1 tránh chia 0 downstream.
- `psi` — yaw tuyệt đối (rad), chỉ dùng cho logging/SLAM phase sau.

State này **khớp** với NMPC 6-state theo paper Kloeser 2020.

### 3.2 Covariance và Noise

```python
self.P = np.diag([0.01, 0.05, 0.01, 0.05, 0.1, 0.1])   # init uncertainty
self.Q = np.diag([1e-4, 1e-3, 1e-4, 1e-3, 5e-4, 1e-4]) # process noise
self.R_cam  = diag([0.01², 0.02²])                     # n, alpha
self.R_full = diag([0.01², 0.02², 0.02²])              # n, alpha, v
```

- P khởi đầu nhỏ cho `n, alpha` (camera tin cậy) và lớn hơn cho rate + `v`.
- Q cân bằng: rate có process noise cao hơn pos, `v` noise trung bình để
  EKF tin IMU ax.
- `R_cam`: 1 cm lateral noise, 0.02 rad heading (~1.1°) — từ tuning bag test.
- `R_full` dùng khi có thêm `v_wheel`.

### 3.3 `predict(ax, ay, yaw_rate, kappa, dt)`

Mô hình theo paper (s-frame, singularity-free):

```python
denom = max(1.0 - n * kappa, 0.1)
s_dot = v * np.cos(alpha) / denom
```
- `s_dot = v·cos(α) / (1 − n·κ)` — đạo hàm arc-length path.
- `max(..., 0.1)` tránh chia 0 khi `n·κ → 1` (xe bám rất gần singularity).

```python
x_new = [
    n + n_dot·dt,
    n_dot + ay·dt,
    alpha + alpha_dot·dt,
    yaw_rate − κ·s_dot,          # alpha_dot new (ĐẶT TRỰC TIẾP, không integrate)
    clip(v + ax·dt, 0, 2.0),
    psi + yaw_rate·dt,
]
```

Lưu ý:
- `alpha_dot` không integrate mà **đặt** `= yaw_rate − κ·s_dot`. Đây là quan
  hệ vật lý: `α̇ = ψ̇ − κ·ṡ` — heading rate của xe so với tangent.
- `v` clip `[0, 2.0]` để tránh IMU drift làm state phát nổ. Upper bound 2 m/s
  > v_max sim 1.5 m/s, đủ margin.

Jacobian A (chỉ row 3 non-trivial do `alpha_dot` phụ thuộc `n, alpha, v`):
```python
d_adot_dn = -kappa * v * cos(alpha) * kappa / denom²
d_adot_da =  kappa * v * sin(alpha) / denom
d_adot_dv = -kappa * cos(alpha) / denom
```

Các entry khác của A: `A[0,1]=1`, `A[2,3]=1` (integrator). Các biến khác
xem như hằng trong `dt` (sai số nhỏ, absorb vào Q).

```python
F = I + dt·A
P = F·P·Fᵀ + Q
```
Discrete EKF predict chuẩn.

### 3.4 `update(z, H, R)` — Joseph form

```python
y = z - H @ self.x
S = H @ self.P @ H.T + R
K = self.P @ H.T @ np.linalg.inv(S)
self.x = self.x + K @ y
I_KH = np.eye(6) - K @ H
self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
```

Joseph form (`(I-KH)P(I-KH)ᵀ + KRKᵀ`) thay vì `(I-KH)P` — **numerical
stability**, tránh P mất tính PSD do floating-point.

### 3.5 `is_healthy()`

```python
diag = np.diag(self.P)
return np.all(diag < 100.0) and np.all(diag > 0.0)
```

Check divergence đơn giản — covariance > 100 hoặc âm (PSD bị phá) → unhealthy.
MPC có thể check flag này để fallback (hiện chưa implement phía MPC).

### 3.6 ROS2 callbacks

**`imu_cb`**: cache ax, ay, yaw_rate. Set `imu_ok=True` + timestamp cho
timeout check.

**`lane_cb`**:
```python
self.kappa = 0.70 * self.kappa + 0.30 * msg.kappa
```
EMA kappa α=0.3 — camera kappa nhiễu, smooth trước khi dùng trong predict.
**Khác** với kappa horizon MPC (dùng raw coeff để tính per-stage).

Update camera:
- Có `v_wheel` → `update_camera_and_velocity([e_y, e_psi, v_wheel])`.
- Chưa có → `update_camera([e_y, e_psi])`.

**`joint_cb`**:
```python
vl = vel_map.get('rear_left_wheel_joint',  None)
vr = vel_map.get('rear_right_wheel_joint', None)
omega_avg = (abs(vl) + abs(vr)) / 2.0
self.v_wheel = omega_avg * WHEEL_RADIUS
```
- Dùng `abs()` để xử lý trường hợp Gazebo trả omega âm (khi quay ngược).
  HW thật luôn dương nếu xe tiến. Lưu ý: KHÔNG detect lùi — ok cho phase 1.

**`enabled_cb`**:
```python
if not prev and self.car_enabled:
    self.ekf.reset()
    self.kappa = 0.0
```
Reset EKF khi xe được ENABLE (False→True). Quan trọng khi respawn xe trong
sim — nếu không reset, state còn giữ giá trị cũ → MPC hành xử sai.

### 3.7 Timer `ekf_cb` @ 100 Hz

```python
imu_timeout = (now - last_imu_time).nanoseconds * 1e-9 > 1.0
if not self.imu_ok or imu_timeout:
    self.ekf.predict(0.0, 0.0, 0.0, self.kappa, self.dt)
else:
    self.ekf.predict(self.ax, self.ay, self.yaw_rate, self.kappa, self.dt)
```

IMU timeout 1 s → propagate với zero input. Không publish zero velocity —
vẫn publish full state, nhưng P sẽ phình ra → `ekf_healthy = False`. Đây
là design an toàn hơn là "fail loud" (crash node).

Publish `VehicleState`: copy state + std + health flag.

---

## 4. TOÁN HỌC

- s-frame derivative: `ṡ = v·cos(α) / (1 − n·κ)`.
- Heading error dynamics: `α̇ = ψ̇ − κ·ṡ` = yaw_rate trừ đi "rotation needed
  to follow curvature".
- EKF time-update: `x_{k+1} = f(x_k, u_k) + w`, `P = FPFᵀ + Q`.
- EKF measurement-update (Joseph): stable form of `P = (I-KH)P`.

---

## 5. THAM SỐ

Không đọc yaml — **hardcoded** trong file. Khi thay xe (đổi wheel radius,
wheelbase) phải sửa trực tiếp ở top-file (`WHEEL_RADIUS`, `LF`, `LR`, `L`).
TODO: externalize ra `robot.yaml`.

---

## 6. GOTCHAS

- **`self.x[4] = 0.1` seed**: nếu init về 0.0, công thức `s_dot = v·cos(α)/
  denom` luôn = 0 → `alpha_dot = yaw_rate`, không có coupling với curvature.
  Seed 0.1 m/s để filter khởi động đúng hướng.
- **`np.clip(v, 0, 2.0)`**: hardcoded upper bound. Nếu v_max tăng >2 m/s
  phải sửa.
- **`abs(vl), abs(vr)`** không phân biệt tiến/lùi — nếu MPC cho `D<0` (phanh
  cứng) Gazebo có thể quay bánh ngược nhưng EKF vẫn thấy v>0.
- **Update cam chạy TRƯỚC predict tại step tiếp theo**: ROS2 Python callback
  không đồng bộ — thứ tự phụ thuộc khi sensor đến. Không hẳn là bug nhưng
  khác chuẩn "predict rồi update".
- **`kappa` EMA trong `lane_cb`** khác với kappa dùng trong MPC (per-stage).
  Nếu debug bất khớp, check 2 topic riêng.
- **H_cam, H_full hardcoded** — nếu đổi state order (ví dụ thêm slip angle)
  phải sửa.
- **Không có `ekf_cb` chạy update**, chỉ predict. Update nằm trong callback
  sensor — đúng kiểu async EKF, nhưng cần nhớ để debug khi sensor drop.

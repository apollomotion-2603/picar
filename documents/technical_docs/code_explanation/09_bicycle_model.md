# 09 — bicycle_model.py

File: `src/mpc_pkg/mpc_pkg/bicycle_model.py` (145 dòng)

---

## 1. MỤC ĐÍCH

Xây dựng mô hình động lực học 6-state **spatial bicycle model** theo paper
Kloeser 2020 bằng CasADi symbolic, sẵn sàng cho acados auto-diff và code
generation.

- Không phải ROS node.
- Trả về `(model, constraint)` — `SimpleNamespace` chứa biểu thức CasADi
  và bounds. File 08 (`acados_settings.py`) consume.

---

## 2. CẤU TRÚC FILE

- Import CasADi `from casadi import *` (MX, vertcat, cos, sin, tanh,
  Function).
- 1 function: `bicycle_model(m, C1, C2, Cm1, Cm2, Cr0, Cr2, cr3, *bounds)`.

---

## 3. GIẢI THÍCH KHỐI CODE

### 3.1 Symbolic state, control, xdot, param

```python
s, n, alpha, v, D, delta = MX.sym(...)   # 6 states
x = vertcat(s, n, alpha, v, D, delta)

derD, derDelta = MX.sym(...)              # 2 controls
u = vertcat(derD, derDelta)

sdot, ndot, alphadot, vdot, Ddot, deltadot = MX.sym(...)
xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot)

kappa_p = MX.sym("kappa_p")
p = vertcat(kappa_p)                      # runtime param (1-dim)
```

- Ý nghĩa:
  - `s` arc-length dọc centerline (m).
  - `n` lateral offset (m, + bên trái theo paper).
  - `α` heading error relative tangent (rad).
  - `v` speed (m/s).
  - `D` throttle command normalized [−1, 1].
  - `δ` steering angle (rad).
- Runtime param `κ_p`: curvature tại stage, **không đưa vào state** (khác
  với cài nhanh khác có thể integrate κ). Lý do: solver assume κ hằng trong
  mỗi stage, thay đổi giữa các stage qua parameter, giữ model compact.

### 3.2 Traction force model (Kloeser §4.2)

```python
Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(cr3 * v)
```
Phân tích 3 thành phần:
- `(Cm1 − Cm2·v)·D`: lực kéo — motor constant giảm theo tốc độ.
  - `Cm1 = 0.28` (N khi v=0, D=1).
  - `Cm2 = 0.05` (N·s/m) giảm lực khi v tăng.
- `Cr2·v²`: drag khí động học (Newton / m²·s²).
- `Cr0·tanh(cr3·v)`: rolling friction, dùng `tanh` smooth thay vì `sign(v)`
  để CasADi differentiable (cần cho Gauss-Newton).
  - `cr3 = 5.0` controls độ steep của tanh.

Kết quả: `Fxd` (N) — lực dọc trục xe thực tế sau khi đã trừ tổn hao.

### 3.3 Spatial dynamics (paper Eq. 10)

```python
sdota = (v * cos(alpha + C1 * delta)) / (1 - kappa_p * n)

f_expl = vertcat(
    sdota,                                    # ṡ
    v * sin(alpha + C1 * delta),              # ṅ
    v * C2 * delta - kappa_p * sdota,         # α̇
    Fxd / m * cos(C1 * delta),                # v̇
    derD,                                     # Ḋ = u₀
    derDelta,                                 # δ̇ = u₁
)
```

Giải thích vật lý:
- `ṡ = v·cos(α + C1·δ) / (1 − κ·n)` — speed projected dọc centerline, hệ số
  `(1 − κ·n)` compensate cho việc xe không nằm đúng trên đường (offset n
  làm arc-length giảm/tăng khi cua).
  - **Singularity**: khi `κ·n → 1` → denominator → 0 → infinite `ṡ`. Paper
    gọi đây là "singularity-free" vì ngay cả `n` gần centerline và `κ` nhỏ
    vẫn OK. Trong code EKF (file 04) có `max(1 − n·κ, 0.1)` để tránh chia
    0 phía ROS side; **phía CasADi không có clamp** — solver xử lý qua
    constraint `|n| ≤ n_max`.
- `ṅ = v·sin(α + C1·δ)` — vận tốc lateral.
- `α̇ = v·C2·δ − κ·ṡ` — heading rate trừ curvature-induced rotation.
  - `C2 = 15.5 rad/m` là 1/wheelbase hiệu dụng scaled (paper convention).
- `v̇ = Fxd/m · cos(C1·δ)` — gia tốc dọc (N/kg).
  - `cos(C1·δ)` projected theo hướng xe (khi δ lớn thì chỉ component
    cos(C1·δ) đẩy xe tiến).
- `Ḋ, δ̇` đơn giản bằng control inputs → control là **derivative** của
  actuator state → smoothness tự động.

**C1, C2 là gì?** Tham số không thứ nguyên paper convention:
- `C1 = lr / (lr + lf) = 0.5` (với lr = lf = 0.11, tỉ lệ slip).
- `C2 = 1 / (lr + lf) = 1/0.22 ≈ 4.54` — **nhưng trong code đặt 15.5**.
  Giá trị 15.5 này khác paper vì Nhã đã tune cho dynamics sim Gazebo. Thực
  chất C2 đóng vai trò gain heading rate per `v·δ`. Không tương ứng trực
  tiếp với wheelbase vật lý.

### 3.4 Nonlinear constraint expressions

```python
a_lat  = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
a_long = Fxd / m
```
- `a_lat`: gia tốc ngang, 2 component:
  - `C2·v²·δ` — centripetal (curvature bán kính 1/(C2·δ)).
  - `Fxd·sin(C1·δ)/m` — component lateral của traction khi bẻ lái.
- `a_long`: gia tốc dọc = Fxd/m.

```python
constraint.expr = vertcat(a_long, a_lat, n, D, delta)
```
Vector constraint `h(x, u) = [a_long, a_lat, n, D, δ]`. file 08 set
`lh/uh` cho 5 phần tử này.

### 3.5 Model bounds → namespace

```python
model.n_min = n_min; model.n_max = n_max
model.throttle_min = ...; model.throttle_max = ...
model.delta_min = ...; model.delta_max = ...
model.ddelta_min = ...; model.dthrottle_min = ...
```
Lưu lại vào namespace để `acados_settings.py` đọc khi set constraint. Phân
biệt: `model.*_min/max` là giới hạn **state/control**, `constraint.*_min/max`
là giới hạn cho `h(x,u)`.

### 3.6 Initial state + Function wrap

```python
model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
constraint.alat = Function("a_lat", [x, u, p], [a_lat])
```
- `x0` zeros — warm-start; MPC node sau đó override `x0[1], x0[2], x0[3]`.
- `constraint.alat` là CasADi Function để debug (không dùng bởi acados, có
  thể evaluate numerically nếu cần).

### 3.7 Model struct output

```python
model.f_expl_expr = f_expl
model.f_impl_expr = xdot - f_expl
model.x    = x
model.xdot = xdot
model.u    = u
model.z    = z          # empty
model.p    = p          # [kappa_p]
model.name = "Spatialbicycle_model"
model.params = params   # numeric params namespace for debug
```
- `f_impl_expr = xdot − f_expl`: implicit form cho acados (khi integrator
  cần DAE). Acados chọn explicit (ERK) nên thực chất dùng `f_expl_expr`.
- `model.p` 1-dim — acados expect `parameter_values` dimension khớp, chỉ
  truyền 1 scalar κ.

---

## 4. TOÁN HỌC

Model đầy đủ (Kloeser 2020 Eq. 10):

```
ṡ      = v·cos(α + C₁·δ) / (1 − κ·n)
ṅ      = v·sin(α + C₁·δ)
α̇      = v·C₂·δ − κ·ṡ
v̇      = (Fxd/m)·cos(C₁·δ)
Ḋ      = u₀
δ̇      = u₁

Fxd    = (Cm1 − Cm2·v)·D − Cr2·v² − Cr0·tanh(cr3·v)
a_lat  = C₂·v²·δ + (Fxd/m)·sin(C₁·δ)
a_long = Fxd/m
```

Liên hệ với Ackermann:
- `ψ̇ (yaw rate) = v·C₂·δ` (xấp xỉ, cho δ nhỏ).
- `R (bán kính) = 1 / (C₂·δ)`.
- Singularity frame: xảy ra khi `n = 1/κ` (xe nằm đúng tâm curvature) —
  bounded bởi `|n| ≤ n_max = 0.2 m` và `|κ| ≤ 0.8` → `|n·κ| ≤ 0.16 < 1` OK.

---

## 5. THAM SỐ

Chia 2 nhóm:

**Dynamic (từ yaml, chú ý tune sim ≠ HW):**
| Tham số | Default | Ý nghĩa |
|---|---|---|
| `m` | 1.5 kg | Khối lượng (sim). HW thật 2.5 kg — xem CLAUDE.md. |
| `C1` | 0.5 | Slip distribution (lr/(lr+lf)) |
| `C2` | 15.5 | Gain heading rate (tuned) |
| `Cm1` | 0.28 | Motor constant |
| `Cm2` | 0.05 | Motor viscous |
| `Cr0` | 0.006 | Rolling resistance |
| `Cr2` | 0.011 | Aero drag |
| `cr3` | 5.0 | Tanh smoothness |

**Bounds**: đã documented ở file 06 + 08.

---

## 6. GOTCHAS

- **C2 ≠ 1/wheelbase**: nhiều người đọc paper gán `C2 = 1/(lf+lr) = 4.54`,
  nhưng code đặt `15.5` (tune empirical). Nếu debug heading không khớp
  Gazebo, cần rerun system identification.
- **m = 1.5 kg sim** vs **m = 2.5 kg HW**: khi deploy phải sửa cả yaml +
  rebuild solver (`rm -rf nmpc_build_6state/`).
- **Singularity `1 − κ·n`**: CasADi không có clamp. Nếu `|n| > 1/|κ|`,
  solver infeasible. Bounds `n_max=0.2` + `kappa_max=0.8` đảm bảo an toàn.
- **`constraint.alat` Function thừa**: chỉ để debug, acados tự tính từ
  `constraint.expr`.
- **`C1·δ` coupling**: tất cả công thức dùng `α + C1·δ` (slip-corrected
  heading). Khi đổi C1, cần hiểu ảnh hưởng lan ra cả ṅ, ṡ, a_lat.
- **Runtime param `kappa_p` 1-dim**: muốn thêm param (ví dụ mass biến thiên)
  phải sửa cả file này + `acados_settings.py` + cách `mpc_node` `set(j, "p", ...)`.
- **Smooth tanh thay sign**: quan trọng cho auto-diff, không dùng
  `sign(v)` vì không differentiable tại v=0.
- **Paper reference**: Kloeser 2020 *NMPC for Racing Using a Singularity-Free
  Path-Parametric Model* — full derivation ở đó.

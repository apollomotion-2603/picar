# 03. Bộ điều khiển NMPC 6 Trạng thái

Tài liệu giải thích thuật toán Nonlinear Model Predictive Control (NMPC)
dựa trên mô hình Spatial Bicycle 6-state (mở rộng Kloeser 2020).

---

## 1. Mô hình toán học (Spatial Bicycle Model)

### 1.1 State và Control

- **States (x):** $[s,\ n,\ \alpha,\ v,\ D,\ \delta]$ — 6 chiều.
  - $s$ — arc-length (progress dọc đường).
  - $n$ — lateral error (sai lệch ngang).
  - $\alpha$ — heading error.
  - $v$ — forward velocity.
  - $D$ — duty cycle throttle $\in[-1,1]$.
  - $\delta$ — góc lái.

- **Controls (u):** $[\dot D,\ \dot\delta]$ — 2 chiều.
  Đưa `D` và `δ` vào state giúp bộ điều khiển **quản lý tốc độ thay đổi**
  của actuator (rate limit) → servo/ESC hoạt động mượt, tránh sốc cơ khí.

- **Runtime parameter (p):** $\kappa_p$ (track curvature tại stage hiện
  tại) — `acados_solver.set(j, "p", [kappa_j])` trước mỗi solve.

### 1.2 Kinematics (singularity-free)

$$
\begin{aligned}
\beta   &= \tfrac{l_r}{l_f+l_r} \delta \\
\dot s  &= v\,\cos(\alpha+\beta) / (1 - n\,\kappa_p) \\
\dot n  &= v\,\sin(\alpha+\beta) \\
\dot\alpha &= \tfrac{v}{l_r}\sin\beta - \kappa_p\dot s
\end{aligned}
$$

### 1.3 Động lực học dọc (traction force)

$$
F_{xd}(v, D) = (C_{m1} - C_{m2} v)\,D - C_{r2} v^2 - C_{r0}\tanh(c_{r3} v)
$$

$$
\dot v = F_{xd}/m,\quad \dot D = \text{derD},\quad \dot\delta = \text{derDelta}
$$

Tham số mặc định: $m=1.5$ kg, $C_{m1}=0.28$, $C_{m2}=0.05$, $C_{r0}=0.006$,
$C_{r2}=0.011$, $c_{r3}=5.0$.

---

## 2. Giải thuật Solver (acados)

| Option | Value |
|--------|-------|
| NLP solver | `SQP_RTI` (Real-Time Iteration, 1 iter/solve) |
| QP solver | `PARTIAL_CONDENSING_HPIPM` |
| Hessian | `GAUSS_NEWTON` |
| Integrator | `ERK` (Explicit Runge-Kutta) |
| Sim stages | 4 |
| Sim steps | 3 |
| Horizon $N$ | 20 steps |
| $T_f$ | 1.0 s (dt = 50 ms) |
| Control rate | 20 Hz |
| Solve time | 1–5 ms (margin 45 ms) ✅ |

---

## 3. Nội suy Curvature $\kappa(s)$

Đây là điểm cải tiến quan trọng giúp xe "biết trước" cua.

Tại mỗi control tick:

1. Đọc cubic coefficients $[a,b,c,d]$ từ `LaneState` gần nhất.
2. Với mỗi stage $j \in [0, N]$:
   - $s_j = \min(v \cdot j \cdot dt,\ s_{max})$ — điểm tham chiếu
     dựa trên vận tốc hiện tại.
   - $x_c'(s_j) = 3as_j^2 + 2bs_j + c$.
   - $x_c''(s_j) = 6as_j + 2b$.
   - $\kappa_j = \text{clip}\!\left(\dfrac{x_c''}{(1+x_c'^2)^{1.5}},\ \pm \kappa_{max}\right)$.
3. Set `solver.set(j, "p", [kappa_j])` cho từng stage.

**Issue mở:** per-stage EMA smoothing giữa các frames không hợp lệ vì
$s_j$ phụ thuộc $v$ → stage $j$ đại diện arc-length khác nhau khi $v$
thay đổi. Đề xuất rollback sang scalar EMA trên $\kappa$ tại $s=0$ rồi
broadcast constant toàn horizon (trade-off: mất preview nhưng stable).
Chi tiết: [../state/05_status_issues.md](../state/05_status_issues.md).

---

## 4. Hàm Mục tiêu (Cost Function — LINEAR_LS)

Residual stage: $y = [x;u] \in \mathbb{R}^{8}$.
Residual terminal: $y_e = x \in \mathbb{R}^{6}$.

$$
Q = \text{diag}(10^{-1},\ 50,\ 10,\ 10^{-1},\ 10^{-3},\ 5\cdot 10^{-3})
$$

$$
R = \text{diag}(10^{-3},\ 5\cdot 10^{-3}),\quad
Q_e = \text{diag}(0.5,\ 100,\ 20,\ 0.1,\ 5\cdot 10^{-3},\ 2\cdot 10^{-3})
$$

$$
W = \tfrac{N}{T_f}\,\text{blkdiag}(Q, R),\quad W_e = \tfrac{T_f}{N}Q_e
$$

**Intent lane-following (không phải racing):**
- Phạt $n$ nặng (50) → bám tâm lane.
- Phạt $\alpha$ vừa (10) → song song hướng lane.
- Phạt $v$ nhẹ (0.1) → track adaptive v_ref.
- Regularise $D, \delta, \dot D, \dot\delta$.

**Reference per-stage:**

$$
y_{ref}^j = [s_j^{ref},\ 0,\ 0,\ v_{ref}^j,\ 0,\ 0,\ 0,\ 0]
$$

- $s^{ref}_j = j \cdot s_{max}/N$ — progress target.
- $v^j_{ref} = \text{clip}\!\left(\dfrac{v_{target}}{1+k_f|\kappa_j|},\
  v_{min\_curve},\ v_{max}\right)$ — adaptive speed.

---

## 5. Ràng buộc (Constraints)

### 5.1 Tuyến tính

- `derD` ∈ [-dthrottle_max, +dthrottle_max] = [-10, +10].
- `derDelta` ∈ [-ddelta_max, +ddelta_max] = [-2, +2] rad/s.
- `n` loose bound $[-12, 12]$ — chỉ để tránh infeasibility cực đoan;
  bound thực nằm trong nonlinear constraint.

### 5.2 Phi tuyến (`con_h_expr`)

$$
\begin{aligned}
a_{long} &\in [-a_{long,\max},\ +a_{long,\max}] \\
a_{lat}  &\in [-a_{lat,\max},\ +a_{lat,\max}] \\
n        &\in [-n_{max},\ +n_{max}] \\
D        &\in [D_{min},\ D_{max}] \\
\delta   &\in [-\delta_{max},\ +\delta_{max}]
\end{aligned}
$$

**Slacked:** chỉ số 0 ($a_{long}$) và 2 ($n$) có slack penalty $z=100$ →
vi phạm nhẹ vẫn feasible, tránh solver fail khi xe lệch mạnh.

---

## 6. Velocity Control (Hướng A)

Thay vì override `v` bằng heuristic sau solve, adaptive speed nằm **bên
trong** cost reference (`v_ref_horizon`) → solver tự optimize `D, v` theo
mô hình traction:

1. Build `κ_horizon` → tính `v_ref_horizon[j]` cho mỗi stage.
2. Set `yref[j][3] = v_ref_horizon[j]`.
3. Sau solve: publish `v_cmd = clip(x1[3], 0, v_max)` — **trust solver**
   prediction cho stage 1.

Kết quả: throttle-velocity nhất quán với mô hình, không còn lỗi logic
"override v bằng công thức khác với công thức trong cost".

---

## 7. Fallback an toàn

| Điều kiện | Hành động |
|-----------|-----------|
| `car_enabled = False` | publish (0, 0), skip solve |
| Lane timeout > 1.0 s | publish (0, 0), skip solve |
| Solver status ≠ 0 | giữ `δ` cũ, `v=0`, log warn |

---

## 8. Build cache

Solver build ra `nmpc_build_6state/` (~5 s rebuild, ~30 s từ scratch).
**Khi đổi `nmpc.yaml` có liên quan model (lf, lr, mass, Cm*, alat_max…)
hoặc đổi `bicycle_model.py` → BẮT BUỘC** `rm -rf nmpc_build_6state/`
trước khi chạy lại, nếu không solver sẽ dùng binary cũ.

# NMPC Notes — Eigen-based RTI-SQP với Condensed QP

Dựa trên bài báo: **Kloeser et al., "NMPC for Racing Using a Singularity-Free Path-Parametric Model with Obstacle Avoidance"**, IFAC 2020.

---
## bước đang xử lý:

### Tiến trình ưu tiên tiếp theo

1. **Hoàn thiện chi tiết hàm condense và solve_qp trong nmpc_solver.hpp**
  - condense: build H, g từ trajectory (x̄, ū), Jacobian (A, B), Q, R, QN
  - solve_qp: projected gradient hoặc clamp (box constraints)

2. **Tạo file test_nmpc_solver.cpp**
  - Test offline: cho x0, ref, chạy condense + solve_qp, in ra Δu, kiểm tra cost, constraint

3. **Sau đó mới chuyển sang nmpc_controller.cpp**
  - ROS node, RTI loop, publish /throttle, /steering_angle

**Tóm lại:**
→ Ưu tiên hoàn thiện nmpc_solver.hpp (condense + QP) và test độc lập trước khi tích hợp ROS node.
→ Có thể dùng center line giả lập (đường thẳng/hình tròn) cho đến khi có dữ liệu thật.

## 1. Kiến trúc 4 Modules

```
nodes/nmpc/
├── nmpc_types.hpp      ← Eigen types + constants
├── nmpc_model.hpp      ← Analytic Jacobians (tính tay) + RK4
├── nmpc_solver.hpp     ← Condensing → H,g,A_ineq + qpOASES solve
└── nmpc_controller.cpp ← ROS node (subscribe/publish + main loop)
```

---

## 2. Bài toán NMPC (Eq. 13 bài báo)

### State vector mở rộng (6 states):
$$x = [s,\ n,\ \alpha,\ v,\ D,\ \delta]^T$$

- $s$: tiến trình trên center line (arc length)
- $n$: khoảng cách ngang so với center line
- $\alpha$: heading tương đối ($\psi - \psi^c$)
- $v$: vận tốc
- $D$: duty cycle (throttle)
- $\delta$: steering angle

### Control vector (2 controls):
$$u = [\dot{D},\ \dot{\delta}]^T$$

### Dynamics (Eq. 10a-10d + augmented states):

$$\dot{s} = \frac{v\cos(\alpha + \beta)}{1 - n\kappa^c}$$

$$\dot{n} = v\sin(\alpha + \beta)$$

$$\dot{\alpha} = \frac{v}{l_r}\sin\beta - \kappa^c \dot{s}$$

$$\dot{v} = \frac{F_x^d}{m}\cos\beta$$

$$\dot{D} = u_1 = \dot{D}$$

$$\dot{\delta} = u_2 = \dot{\delta}$$

Với:
- $\beta = \frac{l_r}{l_r + l_f}\delta$ (Eq. 2, small angle approx)
- $F_x^d = (c_{m1} - c_{m2}v)D - c_{r2}v^2 - c_{r0}\tanh(c_{r3}v)$ (Eq. 4)
- $\kappa^c(s)$: curvature của center line

### NLP formulation (Eq. 13):

$$\min \sum_{k=0}^{N-1} \|x_k - x_{k,ref}\|_Q^2 + \|u_k\|_R^2 + \|x_N - x_{N,ref}\|_{Q_N}^2$$

Subject to:
- $x_0 = x_c$ (current state)
- $x_{k+1} = F(x_k, u_k, \Delta t)$ (RK4)
- $\underline{D} \leq D_k \leq \overline{D}$
- $\underline{\delta} \leq \delta_k \leq \overline{\delta}$
- $\underline{n}(s_k) \leq n_k \leq \overline{n}(s_k)$ (track boundaries)
- $|a_\perp(x_k)| \leq a_{\perp,max}$ (lateral acceleration — model validity)

### Reference trajectory (Eq. 12):

$$s_{k,ref} = s_0 + \frac{s_{N,ref}}{N}k, \quad k = 0,...,N$$

$s_{N,ref}$ = tổng progress mong muốn (bài báo dùng 3m).

---

## 3. RTI-SQP (Real-Time Iteration)

### Ý tưởng
Thay vì giải NLP đến hội tụ (cần 3-10 SQP iterations), **chỉ làm 1 bước SQP** mỗi chu kỳ điều khiển. Trajectory cải thiện dần theo thời gian thực.

### 2 pha mỗi chu kỳ:

```
┌── PREPARATION PHASE (trước khi nhận state mới) ──┐
│ 1. Shift trajectory: x̄_k ← x̄_{k+1}              │
│ 2. Linearize: tính A_k, B_k tại (x̄, ū)           │
│ 3. Build condensed QP matrices (H, partial g)     │
└───────────────────────────────────────────────────┘
                         │
         ──── nhận x_c (state mới) ────
                         │
┌── FEEDBACK PHASE (phải nhanh nhất có thể) ────────┐
│ 4. Cập nhật Δx_0 = x_c - x̄_0                     │
│ 5. Hoàn thành g, giải QP → Δu*                    │
│ 6. Apply u*_0 = ū_0 + Δu*_0                       │
└───────────────────────────────────────────────────┘
```

---

## 4. Condensing — Loại bỏ state variables

### Nguyên lý

Khai thác: $\Delta x_k$ xác định hoàn toàn bởi $(\Delta x_0, \Delta u_0,...,\Delta u_{k-1})$:

$$\Delta x_k = \Phi_k \Delta x_0 + \sum_{j=0}^{k-1} \Gamma_{k,j} \Delta u_j + \hat{d}_k$$

Với:
- $\Phi_k = A_{k-1} A_{k-2} \cdots A_0$ (state transition matrix)
- $\Gamma_{k,j} = A_{k-1} \cdots A_{j+1} B_j$ (control-to-state map)
- $\hat{d}_k$ (tích lũy residuals)

### Gom thành ma trận khối

$$\mathcal{G} = \begin{bmatrix}
B_0 & 0 & \cdots & 0 \\
A_1 B_0 & B_1 & \cdots & 0 \\
A_2 A_1 B_0 & A_2 B_1 & \cdots & 0 \\
\vdots & & \ddots & \vdots \\
\Phi_N A_0^{-1} B_0 & \cdots & & B_{N-1}
\end{bmatrix} \in \mathbb{R}^{(N \cdot n_x) \times (N \cdot n_u)}$$

Tính bằng recursion (không cần inverse):

```cpp
G.block(k*NX, k*NU, NX, NU) = B[k];          // diagonal
for (j = 0; j < k; ++j)
    G.block(k*NX, j*NU, NX, NU) = A[k] * G.block((k-1)*NX, j*NU, NX, NU);
```

### Condensed QP

$$\min_{\Delta u} \frac{1}{2} \Delta u^T H \Delta u + g^T \Delta u$$

$$H = \mathcal{G}^T \mathcal{Q} \mathcal{G} + \mathcal{R} \quad (40 \times 40)$$

$$g = \mathcal{G}^T \mathcal{Q} (\Phi \Delta x_0 + \hat{d} - e_{ref})$$

Track bounds condensed thành linear inequality:

$$\underline{n}(s_k) - \bar{n}_k \leq [\mathcal{G}]_{n\text{-rows}} \Delta u + [\Phi]_{n\text{-rows}} \Delta x_0 + \hat{d}_{n,k} \leq \overline{n}(s_k) - \bar{n}_k$$

→ $lbA \leq C \cdot \Delta u \leq ubA$ → qpOASES exact constraints.

---

## 5. Analytic Jacobians (tính tay)

Đặt: $\lambda = \frac{l_r}{l_r + l_f}$, $c_{\alpha\beta} = \cos(\alpha+\beta)$, $s_{\alpha\beta} = \sin(\alpha+\beta)$, $c_\beta = \cos\beta$, $s_\beta = \sin\beta$, $\rho = \frac{1}{1-n\kappa^c}$.

### $\frac{\partial f}{\partial x}$ (6×6):

| | $s$ | $n$ | $\alpha$ | $v$ | $D$ | $\delta$ |
|---|---|---|---|---|---|---|
| $\dot{s}$ | $-vc_{\alpha\beta}\rho^2 n\kappa'$ | $vc_{\alpha\beta}\rho^2\kappa^c$ | $-vs_{\alpha\beta}\rho$ | $c_{\alpha\beta}\rho$ | $0$ | $-vs_{\alpha\beta}\lambda\rho$ |
| $\dot{n}$ | $0$ | $0$ | $vc_{\alpha\beta}$ | $s_{\alpha\beta}$ | $0$ | $vc_{\alpha\beta}\lambda$ |
| $\dot{\alpha}$ | (via $\dot{s}$) | (via $\dot{s}$) | (via $\dot{s}$) | $\frac{s_\beta}{l_r} - \kappa^c\frac{\partial\dot{s}}{\partial v}$ | $0$ | $\frac{v\lambda c_\beta}{l_r} - \kappa^c\frac{\partial\dot{s}}{\partial\delta}$ |
| $\dot{v}$ | $0$ | $0$ | $0$ | $\frac{(-c_{m2}D - 2c_{r2}v - c_{r0}c_{r3}\text{sech}^2(c_{r3}v))c_\beta}{m}$ | $\frac{(c_{m1}-c_{m2}v)c_\beta}{m}$ | $\frac{-F_x^d s_\beta\lambda}{m}$ |
| $\dot{D}$ | $0$ | $0$ | $0$ | $0$ | $0$ | $0$ |
| $\dot{\delta}$ | $0$ | $0$ | $0$ | $0$ | $0$ | $0$ |

### $\frac{\partial f}{\partial u}$ (6×2):

$$\frac{\partial f}{\partial u} = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & 1 \end{bmatrix}$$

### Euler discretization cho Jacobians:

$$A_k = I + \Delta t \cdot \frac{\partial f}{\partial x}\bigg|_{\bar{x}_k, \bar{u}_k}, \quad B_k = \Delta t \cdot \frac{\partial f}{\partial u}\bigg|_{\bar{x}_k, \bar{u}_k}$$

(Dùng RK4 cho forward simulation, Euler cho Jacobians — đủ chính xác cho RTI)

---

## 6. QP Solver: qpOASES

### Tại sao qpOASES?
- Active-set method → warm start cực tốt (design cho RTI)
- Dense QP 40×40 là sweet spot
- Exact constraint handling (track bounds, lateral accel)
- Box + linear inequality constraints

### Interface:

```cpp
#include <qpOASES.hpp>

// Box constraints: lb ≤ Δu ≤ ub
// Linear inequality: lbA ≤ C·Δu ≤ ubA (track bounds, a_perp)

qpOASES::QProblem qp(N*NU, n_constraints);
qp.init(H.data(), g.data(), C.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);

Eigen::VectorXd du_opt(N*NU);
qp.getPrimalSolution(du_opt.data());
```

---

## 7. Tham số xe Gazebo (từ params.yaml)

| Tham số | Giá trị | Đơn vị |
|---|---|---|
| $l_r$ | 0.11 | m |
| $l_f$ | 0.11 | m |
| $m$ | ~1.5 | kg |
| $c_{m1}$ | 0.287 | |
| $c_{m2}$ | 0.054 | |
| $c_{r0}$ | 0.051 | |
| $c_{r2}$ | 0.0 | |
| $c_{r3}$ | 50.0 | |
| $\delta_{max}$ | 0.6109 | rad |
| $v_{max}$ | 10.0 | m/s |

### NMPC parameters (cần tuning):

| Tham số | Giá trị khởi đầu |
|---|---|
| $N$ (horizon) | 5 |
| $\Delta t$ (sampling) | 50 ms |
| $s_{N,ref}$ | 3.0 m |
| $a_{\perp,max}$ | 4.0 m/s² |
| $Q$ | diag(0.1, 1e-4, 1e-4, 1e-4, 1e-3, 5e-3) |
| $R$ | diag(1e-3, 5e-3) |
| $Q_N$ | diag(5, 100, 1e-4, 1e-4, 1e-3, 5e-3) |

---

## 8. Topic Interface

```
                    NMPC Controller Node
                    ┌────────────────────────────────────────┐
 Subscribes:        │                                        │  Publishes:
 /ground_truth/     │  1. Project state → (s,n,α,v)         │  /throttle (D)
   odom/filtered    │  2. Build reference s_k,ref            │  /steering_angle (δ)
                    │  3. Solve NLP (RTI-SQP)                │
 /line/poly_coeffs  │  4. Extract u*_0 = [Ḋ*, δ̇*]          │
 (optional: lane)   │  5. Integrate D += Ḋ*·Δt, δ += δ̇*·Δt │
                    └────────────────────────────────────────┘
                              │                         │
                              ▼                         ▼
                        drive_model              vehicle_controller
                     (D → F_xᵈ → v_cmd)        (δ → Ackermann → wheels)
```

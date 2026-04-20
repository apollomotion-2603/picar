# 08 — acados_settings.py

File: `src/mpc_pkg/mpc_pkg/acados_settings.py` (187 dòng)

---

## 1. MỤC ĐÍCH

Build OCP (Optimal Control Problem) + solver `AcadosOcpSolver` cho 6-state
bicycle model. Đây là "factory" — gọi 1 lần khi NMPC node khởi động, trả về
`(constraint, model, acados_solver)`. Solver sau đó được NMPC node thao tác
qua `set`/`get`/`solve`.

- Không phải ROS node. Được import bởi `mpc_node.py`.
- Kết quả: generated C code trong `build_dir/` + JSON config.

---

## 2. CẤU TRÚC FILE

- Set env `ACADOS_SOURCE_DIR='~/acados'`.
- Import `AcadosModel, AcadosOcp, AcadosOcpSolver` từ `acados_template`.
- Import `bicycle_model` (file 09).
- 1 function: `acados_settings(Tf, N, build_dir, *dyn_params, *bounds)`.

---

## 3. GIẢI THÍCH KHỐI CODE

### 3.1 ENV + build dir

```python
os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')
...
os.makedirs(build_dir, exist_ok=True)
os.chdir(build_dir)
```
- `ACADOS_SOURCE_DIR` để `acados_template` biết chỗ tìm header + lib. Path
  hardcoded `~/acados` (xem CLAUDE.md — acados v0.3.5 build từ source).
- `os.chdir(build_dir)` — tất cả file generated (C code, Makefile,
  shared library) nằm trong đây. **Side effect**: thay đổi working dir
  process ROS2! Các path tương đối sau đó sẽ sai → mpc_node dùng tuyệt đối.

### 3.2 Build model + constraint

```python
model, constraint = bicycle_model(
    m=..., C1=..., C2=..., Cm1=..., Cm2=...,
    Cr0=..., Cr2=..., cr3=...,
    n_min=-n_max, n_max=n_max,
    throttle_min=..., throttle_max=...,
    delta_min=-delta_max, delta_max=delta_max,
    ddelta_min=-ddelta_max, ddelta_max=ddelta_max,
    dthrottle_min=-dthrottle_max, dthrottle_max=dthrottle_max,
    alat_min=-alat_max, alat_max=alat_max,
    along_min=-along_max, along_max=along_max,
)
```
- Symmetric bounds: `min = -max` cho delta, dthrottle, a_lat, a_long.
- `bicycle_model` return SimpleNamespace cho `model` (dynamics) và
  `constraint` (nonlinear expressions) — chi tiết file 09.

### 3.3 Fill AcadosModel

```python
model_ac = AcadosModel()
model_ac.f_impl_expr = model.f_impl_expr   # implicit DAE (nếu có)
model_ac.f_expl_expr = model.f_expl_expr   # explicit ODE ẋ = f(x,u,p)
model_ac.x    = model.x
model_ac.xdot = model.xdot
model_ac.u    = model.u
model_ac.z    = model.z                    # algebraic vars (none)
model_ac.p    = model.p                    # runtime params: κ (np=1)
model_ac.con_h_expr = constraint.expr      # nonlinear constraint
model_ac.name = model.name
ocp.model = model_ac
```

Mapping từ CasADi symbolic → acados struct. `p` là key: **κ** truyền vào
mỗi stage qua `solver.set(j, "p", np.array([κ_j]))`.

### 3.4 Dimensions

```python
nx   = 6   # x = [s, n, α, v, D, δ]
nu   = 2   # u = [Ḋ, δ̇]
ny   = 8   # stage cost residual = [s, n, α, v, D, δ, Ḋ, δ̇]
ny_e = 6   # terminal cost residual = x
ns   = 2   # số slack variables
nsh  = 2   # số hard constraint được slack hóa
```

### 3.5 Cost — LINEAR_LS weighted

```python
Q = np.diag([1e-1, 5e1, 1e1, 1e-1, 1e-3, 5e-3])
R = np.diag([1e-3, 5e-3])
Qe = np.diag([5e-1, 1e2, 2e1, 1e-1, 5e-3, 2e-3])
```

Trọng số (theo thứ tự state):

| Element | Q stage | Qe terminal | Comment |
|---|---|---|---|
| s | 1e-1 | 5e-1 | Progress nhẹ |
| n (lateral) | **5e1** | **1e2** | Ưu tiên bám tâm |
| α (heading) | 1e1 | 2e1 | Ưu tiên heading |
| v | 1e-1 | 1e-1 | Velocity track nhẹ (đã adaptive) |
| D | 1e-3 | 5e-3 | Regularize throttle |
| δ | 5e-3 | 2e-3 | Regularize steering |
| Ḋ | 1e-3 | — | Smooth throttle |
| δ̇ | 5e-3 | — | Smooth steering |

Comment trong code giải thích triết lý: **Lane following, KHÔNG phải racing**
→ penalize `n, α` mạnh, velocity nhẹ (đã tune qua adaptive v_ref).

### 3.6 Scaling theo horizon

```python
unscale = N / Tf
ocp.cost.W   = unscale * scipy.linalg.block_diag(Q, R)
ocp.cost.W_e = Qe / unscale
```
- `unscale = N/Tf = 20/1.0 = 20`. W stage nhân lên (tổng cost ~ Tf·cost_rate).
- Terminal chia xuống để balance giữa stage cost tổng N và terminal cost đơn.
- Convention acados trick để số học stable khi đổi Tf, N.

### 3.7 Output mapping V_x, V_u

```python
Vx = np.zeros((8, 6)); Vx[:6, :6] = I6         # y[0:6] = x
Vu = np.zeros((8, 2)); Vu[6, 0] = 1; Vu[7, 1] = 1 # y[6:8] = u
Vx_e = np.zeros((6, 6)); Vx_e[:6, :6] = I6      # y_e = x
```
LINEAR_LS: `y = Vx·x + Vu·u`, cost `= |y − yref|²_W`.

### 3.8 Slack — soft constraint

```python
ocp.cost.zl = 100 * np.ones(2); ocp.cost.Zl = 0 * np.ones(2)
ocp.cost.zu = 100 * np.ones(2); ocp.cost.Zu = 0 * np.ones(2)
```
- `zl/zu` linear slack penalty = 100. `Zl/Zu = 0` không có quadratic.
- Chỉ soft 2 constraint (xem `idxsh` dưới).

### 3.9 Constraints

```python
ocp.constraints.lbx   = np.array([-12])
ocp.constraints.ubx   = np.array([ 12])
ocp.constraints.idxbx = np.array([1])         # n ≤ 12 (loose)
```
**Loose box bound** cho `n` (index 1) — range ±12 m, tường đồ thị. Tight
bound thực tế qua nonlinear `con_h` (§ below).

```python
ocp.constraints.lbu   = [dthrottle_min, ddelta_min]
ocp.constraints.ubu   = [dthrottle_max, ddelta_max]
ocp.constraints.idxbu = [0, 1]                # u[0], u[1]
```
Hard box bound cho control rate.

```python
ocp.constraints.lh = [along_min, alat_min, n_min, throttle_min, delta_min]
ocp.constraints.uh = [along_max, alat_max, n_max, throttle_max, delta_max]
```
Nonlinear constraint h(x,u):
- a_long, a_lat (tính từ model, xem file 09).
- n tight bound (±0.20 m).
- throttle D ∈ [−1, 1].
- delta ∈ [−0.6109, 0.6109].

```python
ocp.constraints.lsh   = np.zeros(2)
ocp.constraints.ush   = np.zeros(2)
ocp.constraints.idxsh = np.array([0, 2])
```
Soft-hóa 2 constraint: **idx 0 (a_long) và idx 2 (n tight)**. Dễ hiểu:
khi xe cần vượt mép lane để tránh (DLC), hoặc phanh gắt, slack ≥ 0 cho
phép vi phạm với penalty 100.

### 3.10 Initial condition + runtime param

```python
ocp.constraints.x0 = model.x0             # [0, 0, 0, 0, 0, 0]
ocp.parameter_values = np.array([0.0])    # κ_p khởi tạo
```

### 3.11 Solver options

```python
ocp.solver_options.tf              = 1.0
ocp.solver_options.qp_solver       = "PARTIAL_CONDENSING_HPIPM"
ocp.solver_options.nlp_solver_type = "SQP_RTI"
ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"
ocp.solver_options.integrator_type = "ERK"
ocp.solver_options.sim_method_num_stages = 4
ocp.solver_options.sim_method_num_steps  = 3
ocp.solver_options.print_level     = 0
```
- **PARTIAL_CONDENSING_HPIPM**: QP solver high-performance (Riccati).
- **SQP_RTI**: Real-Time Iteration (1 iter / solve, low latency).
- **GAUSS_NEWTON**: Hessian approximation `J^T J` (paper standard). Warning
  "Gauss-Newton + EXTERNAL" trong acados v0.3.5 bình thường (CLAUDE.md §7).
- **ERK**: Explicit Runge-Kutta, stage 4 step 3 → RK4 × 3 substep mỗi shooting
  interval (Δt = 50 ms → substep 16.7 ms).

### 3.12 Create solver

```python
json_file = os.path.join(build_dir, "acados_ocp_6state.json")
acados_solver = AcadosOcpSolver(ocp, json_file=json_file)
return constraint, model, acados_solver
```
- Generate C code + compile → `.so`, cache trong `build_dir`. Lần sau
  nếu file JSON giống hệt → reuse cache (nhanh). Đổi params nhỏ có khi
  không trigger rebuild → **phải `rm -rf build_dir`** khi đổi model/bounds.

---

## 4. TOÁN HỌC

OCP tổng quát:
```
min_{x, u}  Σ_{j=0}^{N-1} ||y_j - y_ref_j||²_W  +  ||y_N - y_ref_N||²_W_e
                    + penalty(slack)
s.t.   x_{j+1} = Φ(x_j, u_j, p_j)    (RK4 integration of bicycle ODE)
       lbx ≤ x ≤ ubx,  lbu ≤ u ≤ ubu
       lh ≤ h(x, u) ≤ uh   (with slack on idxsh)
       x_0 = given
```

---

## 5. THAM SỐ QUAN TRỌNG

Truyền vào function từ `mpc_node.py`:
- `Tf=1.0`, `N=20` → `dt = 0.05 s`.
- Dynamic params: `m, C1, C2, Cm1, Cm2, Cr0, Cr2, cr3`.
- Bounds: `n_max, delta_max, throttle_*, ddelta_max, dthrottle_max,
  alat_max, along_max`.

Hardcoded trong file:
- Trọng số Q, R, Qe (lane-following tuned).
- Slack penalty = 100.
- RK4 stage=4, step=3.

---

## 6. GOTCHAS

- **`os.chdir(build_dir)` side effect**: thay đổi CWD process ROS2. Sau
  `acados_settings`, `os.getcwd()` = build_dir. Nếu code downstream dùng
  path relative → bug. `mpc_node.py` dùng absolute path nên OK.
- **Rebuild cache**: sửa Q/R/weights trong file này → acados KHÔNG tự
  rebuild, phải `rm -rf nmpc_build_6state/`. Triệu chứng: tune weights
  không effect.
- **Gauss-Newton + EXTERNAL warning**: normal, ignore.
- **`idxbx = [1]` với range ±12 m**: looks wrong but intentional — tight
  bound nằm ở `con_h` với slack.
- **Symmetric bounds**: `n_min = -n_max` bên ngoài truyền vào. Nếu track
  bất đối xứng, phải expose cả 2 bên.
- **`print_level=0`** — tắt output acados. Debug solver cần đổi thành 1/2.
- **Vehicle params truyền từng param riêng lẻ**: dễ quên sync giữa
  `nmpc.yaml`, `acados_settings.py`, `bicycle_model.py`. Refactor thành
  dict sẽ an toàn hơn.
- Paper reference `Nhã's test10/src/race_cars` — Nhã đem codebase racing
  xuống làm base, ta adapt sang lane-following (đổi Q, target, v range).

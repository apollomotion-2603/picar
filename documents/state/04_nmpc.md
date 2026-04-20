# 04. NMPC 6-state — Model, Solver, Cost, Constraints

Updated 2026-04-19.

**Source:** `src/mpc_pkg/mpc_pkg/mpc_node.py`, `acados_settings.py`, `bicycle_model.py`.
**Reference:** Kloeser 2020 — *NMPC for Racing Using a Singularity-Free
Path-Parametric Model* (mở rộng từ 4-state sang 6-state với actuator rate control).

---

## Model

### State & control

```
State  x = [s, n, α, v, D, δ]       (6 dims)
  s : arc-length dọc đường tham chiếu [m]
  n : lateral offset [m]
  α : heading error [rad]
  v : forward velocity [m/s]
  D : duty cycle throttle [-1, 1]
  δ : steering angle [rad]

Control u = [derD, derDelta]         (2 dims, = Ḋ, δ̇)
Runtime param p = kappa_p            (track curvature tại stage hiện tại)
```

### Kinematics (singularity-free spatial)

```
β     = (lr / (lf+lr)) · δ
ṡ     = v · cos(α + β) / (1 − n · κ_p)
ṅ     = v · sin(α + β)
α̇     = (v / lr) · sin(β) − κ_p · ṡ
```

### Traction force model (Fxd drives v̇)

```
Fxd   = (Cm1 − Cm2·v) · D − Cr2·v² − Cr0·tanh(cr3·v)
v̇     = Fxd / m

Ḋ     = derD        (control input)
δ̇     = derDelta    (control input)
```

**Params (yaml):** `mass, Cm1, Cm2, Cr0, Cr2, cr3, C1, C2, lf, lr`.
Default: `m=1.5, Cm1=0.28, Cm2=0.05, Cr0=0.006, Cr2=0.011, cr3=5.0`.

---

## Solver config (acados)

| Option | Value |
|--------|-------|
| `nlp_solver_type` | `SQP_RTI` (Real-Time Iteration, 1 iter/solve) |
| `qp_solver` | `PARTIAL_CONDENSING_HPIPM` |
| `hessian_approx` | `GAUSS_NEWTON` |
| `integrator_type` | `ERK` (Explicit Runge-Kutta) |
| `sim_method_num_stages` | 4 |
| `sim_method_num_steps` | 3 |
| `N` | 20 |
| `Tf` | 1.0 s |
| `ctrl_hz` | 20 Hz (dt = 50 ms) |
| Solve time | 1–5 ms (margin 45 ms vs period 50 ms) ✅ |
| Build dir | `nmpc_build_6state/` (shared all maps) |
| Build time | ~5s rebuild, ~30s từ scratch |

---

## Cost function (LINEAR_LS)

```
ny   = nx + nu = 8   (stage residual: [x; u])
ny_e = nx     = 6   (terminal residual: [x])

Q  = diag([1e-1, 5e1, 1e1, 1e-1, 1e-3, 5e-3])   # s, n, α, v, D, δ
R  = diag([1e-3, 5e-3])                         # derD, derDelta
Qe = diag([5e-1, 1e2, 2e1, 1e-1, 5e-3, 2e-3])   # terminal

unscale = N/Tf = 20
W   = unscale · blkdiag(Q, R)
W_e = Qe / unscale
```

**Lane-following intent (not racing):**
- Phạt `n` nặng (5e1) → bám tâm lane.
- Phạt `α` vừa (1e1) → hướng song song lane.
- Phạt `v` nhẹ (1e-1) → track adaptive v_ref từ κ.
- Phạt `D, δ` rất nhẹ → regularise actuator.
- Phạt `derD, derDelta` nhẹ (1e-3, 5e-3) → mượt lái/ga.

### Per-stage runtime references (`mpc_node.py:318-333`)

```python
for j in range(N):
    solver.set(j, "p", [kappa_horizon[j]])
    yref = [s_horizon[j], 0, 0,
            v_ref_horizon[j], 0, 0, 0, 0]
    solver.set(j, "yref", yref)

# Terminal
solver.set(N, "p", [kappa_horizon[N]])
yref_N = [s_horizon[N], 0, 0, v_ref_horizon[N], 0, 0]
solver.set(N, "yref", yref_N)
```

- `s_horizon = linspace(0, s_max, N+1)` (progress reference).
- `v_ref_horizon[j] = clip(target_v / (1 + kappa_factor·|κ[j]|),
  v_min_curve, v_max)`.

---

## Constraints

| Variable | Range | Ghi chú |
|----------|-------|---------|
| `n` (state 1) | [-12, 12] (loose bound) | Tightened qua `lh`: [-0.20, +0.20] m |
| `derD` (control 0) | [-10, +10] | `dthrottle_max` |
| `derDelta` (control 1) | [-2, +2] rad/s | `ddelta_max` |

### Nonlinear (`con_h_expr`)

```
along ∈ [-along_max, +along_max]   # longitudinal accel (soft)
alat  ∈ [-alat_max, +alat_max]     # lateral accel (soft)
n     ∈ [-n_max, +n_max]           # lateral offset (soft)
D     ∈ [throttle_min, throttle_max]  # duty
δ     ∈ [-delta_max, +delta_max]   # steering
```

**Slacked:** indices 0 (along), 2 (n) với penalty `zl=zu=100`.

---

## κ(s) horizon interpolation (`mpc_node.py:224-261`)

```python
# s_j = v · j · dt   (predicted arc-length tại stage j)
for j in range(N+1):
    s_j = min(v·j·dt, lane_s_max)
    xp  = 3a·s_j² + 2b·s_j + c
    xpp = 6a·s_j + 2b
    κ[j] = clip(xpp / (1+xp²)^1.5, -kappa_max, +kappa_max)

# Per-stage EMA (⚠️ design bug — xem 05_status_issues.md):
κ_smooth[j] = α·κ_smooth_prev[j] + (1-α)·κ[j]
```

---

## Velocity logic (Hướng A — đã apply)

**Initial state** (`mpc_node.py:285-301`):
- `n, α`: EKF nếu healthy, fallback raw perception.
- `v`: EKF `msg.v` nếu healthy, nếu không giữ `current_state[3]` (warm-start).
- `D, δ`: warm-start từ `current_state[4], [5]` (output stage 1 frame trước).

**Command output** (`mpc_node.py:349-360`):
- Đọc solver stage 1: `x1 = solver.get(1, "x")`.
- `delta_cmd = x1[5]`, `D_cmd = x1[4]`.
- `v_cmd = clip(x1[3], 0, v_max)` — trust solver's predicted v.
- Publish `/steering_angle=delta_cmd`, `/velocity=v_cmd`.
- Update `current_state = x1` (warm-start).

**Không còn override v** bằng heuristic `V_MAX/(1+8|κ|)` tại publish — adaptive
v hiện nằm trong cost reference (`v_ref_horizon`), solver tự optimize.

---

## Safety fallbacks

| Condition | Action |
|-----------|--------|
| `car_enabled=False` | publish (0, 0), skip solve |
| Lane timeout > `timeout=1.0s` | publish (0, 0), skip solve |
| Solver status ≠ 0 | hold previous δ, v=0, warn log |

---

## Per-map configs

| Param | `nmpc_map1.yaml` | `nmpc.yaml` (map 3) | Lý do |
|-------|------------------|---------------------|-------|
| `kappa_max` | 2.0 | **0.8** | Map 3 perception spike κ=0.97 → clamp chặt |
| `kappa_ema_alpha` | 0.5 | **0.7** | Map 3 noisy hơn, smooth mạnh hơn |
| `target_v` | 1.0 | 1.0 | same |
| `kappa_speed_factor` | 2.0 | 2.0 | same |
| `solver_build_dir` | `nmpc_build_6state` | same | chia sẻ solver |

---

## Log format

```
n=Xmm α=X.X° v=X.XX(EKF|perc) D=X.XXX δ=X.X° κ₀=X.XXXX vref₀=X.XX v→X.XX t=X.Xms st=0
```

Ví dụ output tốt: `n=+3.2mm α=+1.2° v=0.95(EKF) D=0.147 δ=-2.1° κ₀=-0.0523 vref₀=0.98 v→0.97 t=2.1ms st=0`.

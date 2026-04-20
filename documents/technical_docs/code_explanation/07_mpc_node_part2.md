# 07 — mpc_node.py (Part 2: κ horizon, v_ref, solve loop)

File: `src/mpc_pkg/mpc_pkg/mpc_node.py` dòng 226–407.

---

## 1. MỤC ĐÍCH (ngữ cảnh)

Part 2 giải thích **cách NMPC đọc lane coeffs → xây horizon parameters
(κ_j, v_ref_j) → solve → extract control**. Đây là phần "brain" thực sự
của node.

---

## 2. GIẢI THÍCH KHỐI CODE

### 2.1 `_compute_kappa_horizon()` — κ(s_j) per-stage

```python
if self.lane_coeffs is None:
    return np.zeros(self.N + 1)

a, b, c, _d = self.lane_coeffs
s_max = max(float(self.lane_s_max), 0.05)
ds = s_max / self.N
kappa_arr = np.zeros(self.N + 1)

for j in range(self.N + 1):
    s_j = j * ds
    xp  = 3*a*s_j**2 + 2*b*s_j + c
    xpp = 6*a*s_j + 2*b
    denom = (1 + xp**2)**1.5
    if abs(denom) > 1e-8:
        kappa_arr[j] = np.clip(
            xpp / denom, -self.KAPPA_MAX, self.KAPPA_MAX)
```

- Cubic `x_c(s) = a·s³ + b·s² + c·s + d`. Lấy `x'`, `x''` giải tích.
- Công thức curvature: `κ = x''(s) / (1 + x'(s)²)^(3/2)`.
- Clip `|κ| ≤ KAPPA_MAX (0.8)` để chặn perception outlier (bag 7 từng
  đo `|κ|_max = 0.97` trong khi thực tế ~0.3 — xem comment yaml).
- Sample `N+1 = 21` điểm với `s_j = j · s_max / N`.
- **Arc-length uniform, không phụ thuộc v**. Khác các bản cũ dùng
  `s_j = v·j·dt` (time uniform) — bản cũ khi `v` thay đổi, stage j
  trượt theo đường cong → EMA per-stage không có ý nghĩa.

EMA per-stage:
```python
a_ema = self.KAPPA_EMA_ALPHA   # 0.7
if self.kappa_horizon_smooth is None:
    self.kappa_horizon_smooth = kappa_arr.copy()
else:
    self.kappa_horizon_smooth = (
        a_ema * self.kappa_horizon_smooth + (1.0 - a_ema) * kappa_arr)
return self.kappa_horizon_smooth.copy()
```
- Frame-to-frame smoothing: 70% giá trị cũ + 30% mới.
- Giữ preview 1 s stable, tránh MPC "giật" khi perception κ nhiễu (đặc biệt
  đoạn cuối DLC).

### 2.2 `_compute_v_ref_horizon(kappa_arr)` — adaptive v_ref

```python
v_ref = self.TARGET_V / (1.0 + self.KAPPA_FACTOR * np.abs(kappa_arr))
return np.clip(v_ref, self.V_MIN_CURVE, self.V_MAX)
```

- Công thức: `v_ref(κ) = target_v / (1 + β·|κ|)` với `β = kappa_factor`.
- Khi `κ → 0`: `v_ref → target_v`. Khi `κ` lớn: `v_ref` giảm dần.
- Ví dụ sim: `target_v=1.0`, `β=2.0`. `κ=0.3` → `v_ref = 1/(1+0.6) = 0.625`.
  `κ=0.8` → `v_ref = 1/2.6 = 0.385`.
- Clip `[V_MIN_CURVE=0.3, V_MAX=1.5]`. Floor đảm bảo xe không đứng khựng
  khi cua gắt.
- **Key advantage**: per-stage v_ref nghĩa là solver "nhìn trước" — thấy
  κ lớn ở stage tương lai sẽ giảm D ngay bây giờ (braking proactive).

### 2.3 `control_cb()` — main control loop @ 20 Hz

**Step 1: Safety gates.**
```python
if not self.car_enabled:
    self._publish(0.0, 0.0)
    return

dt_lane = (now - self.last_lane_time).nanoseconds * 1e-9
if dt_lane > self.TIMEOUT or not self.lane_ok:
    self._publish(0.0, 0.0)
    return
```
- Xe disabled hoặc lane lost >1 s → publish zero (fail-safe stop).

**Step 2: Build initial state x0.**
```python
x0 = self.current_state.copy()
x0[0] = 0.0   # s always relative
```
- Reset `s` mỗi frame — spatial model với s tương đối (theo paper).

```python
if self.ekf_healthy and self.ekf_n is not None:
    x0[1] = float(np.clip(self.ekf_n, -self.N_MAX, self.N_MAX))
    x0[2] = float(np.clip(self.ekf_alpha, -self.DELTA_MAX, self.DELTA_MAX))
else:
    x0[1] = self.perc_n
    x0[2] = self.perc_alpha
```
- Prefer EKF, fallback perception thô. Clip vào feasible region.

```python
if self.ekf_healthy and self.ekf_v is not None:
    x0[3] = float(np.clip(self.ekf_v, self.V_MIN, self.V_MAX))
```
- `v` chỉ update từ EKF khi healthy, nếu không giữ giá trị cũ trong
  `current_state`.

`x0[4]=D`, `x0[5]=δ` **không set lại** — dùng prediction stage-1 frame
trước làm warm-start, match đúng với gì xe đang thực thi.

**Step 3: Set initial constraint & per-stage params.**
```python
self.solver.set(0, "lbx", x0)
self.solver.set(0, "ubx", x0)

kappa_horizon = self._compute_kappa_horizon()
v_ref_horizon = self._compute_v_ref_horizon(kappa_horizon)

s_max = float(self.lane_s_max) if self.lane_s_max > 0.05 else 0.5
s_horizon = np.linspace(0.0, s_max, self.N + 1)

for j in range(self.N):
    self.solver.set(j, "p", np.array([kappa_horizon[j]]))
    yref = np.array([s_horizon[j], 0.0, 0.0,
                     v_ref_horizon[j], 0.0, 0.0, 0.0, 0.0])
    self.solver.set(j, "yref", yref)

self.solver.set(self.N, "p", np.array([kappa_horizon[self.N]]))
yref_N = np.array([s_horizon[-1], 0.0, 0.0,
                   v_ref_horizon[self.N], 0.0, 0.0])
self.solver.set(self.N, "yref", yref_N)
```
- `lbx = ubx = x0` ép x_0 bằng initial state (equality via hard bounds).
- Stage j ∈ [0, N−1]: set param `p_j = κ_j`, yref 8-dim
  `[s, n_ref=0, α_ref=0, v_ref_j, D_ref=0, δ_ref=0, derD_ref=0, dδ_ref=0]`.
- Stage N (terminal): yref 6-dim (không có u), `p = κ_N`.
- **`s_horizon` và `κ_horizon` đồng bộ** (cùng `s_max/N` uniform) — MPC
  tracking `s` đúng điểm nơi κ được evaluate.

**Step 4: Solve.**
```python
t0 = time.time()
status = self.solver.solve()
t_ms = (time.time() - t0) * 1000

if status != 0:
    self.get_logger().warn(
        f'NMPC solver status={status}, hold previous command')
    self._publish(float(self.current_state[5]), 0.0)
    return
```
- `solver.solve()` RTI single iteration (không loop như SQP full).
- Status 0 = OK. Khác 0 (1/2/3/4) → log warn, giữ δ cũ, v=0 (phanh cứng).
- `t_ms` log để monitor computation time (target <10 ms @ 20 Hz).

**Step 5: Extract control from stage-1 prediction.**
```python
x1 = self.solver.get(1, "x")
D_cmd     = float(x1[4])
delta_cmd = float(x1[5])
v_cmd = float(np.clip(x1[3], 0.0, self.V_MAX))

self.current_state = x1.copy()
self._publish(delta_cmd, v_cmd)
```
- Output **stage-1** state (không phải stage-0 = current) vì control
  được apply tại period tiếp theo → state xe sẽ đạt sau 1 `dt = 50 ms`.
- Publish `δ` và `v` (không publish `D` — controller plugin nhận signed
  velocity setpoint).
- `v_cmd` clip `≥ 0` để không lùi.
- `current_state = x1` → frame sau dùng làm warm-start cho `D, δ`.

**Step 6: Logging verbose.**
```
n=12.3mm α=1.2° v=0.85(EKF) D=0.35 δ=2.1° κ₀=0.012 vref₀=0.95 v→0.85 4.2ms st=0
```
Format này khớp với CLAUDE.md §5 Code style. Source velocity `(EKF)` hoặc
`(perc)` để biết fallback kích hoạt chưa.

### 2.4 `_publish(delta, v)`

```python
msg_d = Float64(); msg_d.data = delta
msg_v = Float64(); msg_v.data = v
self.pub_delta.publish(msg_d)
self.pub_vel.publish(msg_v)
```
Trivial publisher. Không kèm timestamp (Float64 không có header).

---

## 3. TOÁN HỌC

- Curvature cubic: `κ(s) = (6as + 2b) / (1 + (3as² + 2bs + c)²)^(3/2)`.
- Adaptive v_ref: `v_ref(κ) = v_target / (1 + β·|κ|)`, sau clip `[v_min_c, v_max]`.
- EMA: `y_k = α·y_{k-1} + (1−α)·x_k`, α=0.7.
- RTI (Real-Time Iteration): 1 QP iteration per control period. Trade-off
  sub-optimality vs latency.

---

## 4. THAM SỐ QUAN TRỌNG

Bên cạnh Part 1:
- `KAPPA_MAX = 0.8`: clip curvature input.
- `KAPPA_EMA_ALPHA = 0.7`: 70% smoothing.
- `KAPPA_FACTOR = 2.0`, `V_MIN_CURVE = 0.3`, `TARGET_V = 1.0` (Map 3 DLC).
- Map 1 oval dùng `nmpc_map1.yaml` với giá trị khác (xem file yaml).

---

## 5. KNOWN ISSUES / GOTCHAS

- **Đổi `kappa_*` không rebuild solver cần thiết**: các param này dùng
  **runtime** trong Python, không vào C code acados. Chỉ khi đổi
  constraint/cost/model mới cần `rm -rf build_dir`.
- **Status != 0 fallback publish `v = 0`**: phanh cứng có thể làm xe dừng
  đột ngột. Alternative: giảm dần v_cmd. Hiện tại design ưu tiên an toàn.
- **`s_horizon = np.linspace(0, s_max, N+1)`**: khoảng cách stage
  `= s_max / N`. Nếu `v·dt > s_max/N` trong 1 period → stage 1 chưa "tới"
  điểm xe thực sự đến → lag. Với `s_max=0.7`, `N=20`: `ds=35 mm`, `v·dt`
  = `1·0.05 = 50 mm` ở v=1. Xe đang "nhanh hơn horizon" ở v cao → tune
  `N` hoặc `s_max` nếu track dài.
- **Warm-start `D, δ` từ stage-1 frame trước** rất quan trọng cho RTI
  convergence. Nếu reset bừa → solver cần 2-3 frame để ổn định.
- **log `I` mỗi frame** (20 Hz) khá spam — dùng `--log-level WARN` khi demo.
- **Không có `kappa_raw` publishing** — debug khó nếu chỉ có ROSbag
  `/perception/lane_state` mà không có nội bộ EMA state.

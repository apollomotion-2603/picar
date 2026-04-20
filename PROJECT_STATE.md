# PROJECT_STATE — RC Car Autonomous Driving

Updated 2026-04-19. Apollomotion (Phi) + Nhã | ĐH GTVT TP.HCM.

File này là **index** cho technical state. Chi tiết ở `documents/state/`.
Kế hoạch phase ở `documents/plan/`. Conventions + identity ở `CLAUDE.md`.

---

## 1. Quick facts

- **Sim stack:** ROS2 Humble + Ignition Gazebo 6 Fortress + acados v0.3.5.
- **NMPC:** 6-state spatial bicycle (Kloeser 2020 extended), SQP_RTI + HPIPM.
- **Control rate:** 20 Hz (N=20, Tf=1.0s, dt=50ms).
- **Pipeline:** camera → perception → EKF → NMPC → `/velocity` + `/steering_angle`.
- **Maps:** 1=oval ✅ · 2=chicane ~ (drop) · 3=DLC ✅ (baseline bag 11) · 4=obstacle ~ (drop).
- **Status 2026-04-19:** Sim closed → chuyển sang hardware deploy + thesis.

---

## 2. Workspace layout

```
~/main/1_projects/1_autonomous_car_research/ros2_ws/
├── CLAUDE.md                 ← AI entry (identity + conventions)
├── PROJECT_STATE.md          ← THIS FILE (technical index)
├── README.md
├── src/
│   ├── gazebo_ackermann_steering_vehicle/   sim + URDF + launch
│   ├── perception_pkg/                      BEV + lane detection
│   ├── ekf_pkg/                             EKF + reset_node
│   ├── mpc_pkg/                             NMPC 6-state + acados
│   └── lane_msgs/                           custom msgs
├── nmpc_build_6state/        solver cache (delete to rebuild)
├── data/bags/                rosbag recordings + analyze scripts
└── documents/
    ├── plan/                 phase plans
    ├── state/                technical state detail
    └── technical_docs/       thesis-style writeup
```

---

## 3. Sub-docs (state detail — mỗi file ≤200 dòng)

| File | Nội dung |
|------|----------|
| [01_pipeline.md](documents/state/01_pipeline.md) | Packages, nodes, topics, custom messages |
| [02_launch_commands.md](documents/state/02_launch_commands.md) | Launch, build, reset, service commands |
| [03_perception.md](documents/state/03_perception.md) | Perception pipeline + BEV + sliding window |
| [04_nmpc.md](documents/state/04_nmpc.md) | NMPC 6-state model + acados cost/constraints |
| [05_status_issues.md](documents/state/05_status_issues.md) | Phase progress, completed, known issues |

---

## 4. Sub-docs (plan)

| File | Nội dung |
|------|----------|
| [01_phase_current.md](documents/plan/01_phase_current.md) | Phase 1.5a + 1.5b chi tiết |
| [02_phase_future.md](documents/plan/02_phase_future.md) | Phase 2 / 3 / 4 / 5 |
| [03_hardware.md](documents/plan/03_hardware.md) | Hardware architecture + specs |

---

## 5. Sub-docs (technical / thesis-style)

| File | Nội dung |
|------|----------|
| [01_ARCHITECTURE_OVERVIEW.md](documents/technical_docs/01_ARCHITECTURE_OVERVIEW.md) | High-level data flow |
| [02_PERCEPTION_AND_EKF.md](documents/technical_docs/02_PERCEPTION_AND_EKF.md) | Perception + EKF |
| [03_NMPC_CONTROL_LOGIC.md](documents/technical_docs/03_NMPC_CONTROL_LOGIC.md) | NMPC algorithm |
| [04_WORKFLOW_AND_TUNING.md](documents/technical_docs/04_WORKFLOW_AND_TUNING.md) | Vận hành + tuning |

---

## 6. Critical parameters (snapshot)

### NMPC 6-state (`src/mpc_pkg/config/nmpc.yaml` for Map 3)

| Param | Value | Note |
|-------|-------|------|
| N | 20 | horizon steps |
| Tf | 1.0 s | horizon time |
| ctrl_hz | 20 Hz | dt = 50 ms |
| target_v | 1.0 m/s | cost reference |
| v_max | 1.5 m/s | hard bound |
| delta_max | 0.6109 rad | ≈ 35° |
| n_max | 0.20 m | lateral offset bound |
| kappa_max | **0.8** (map3) / **2.0** (map1) | κ clamp |
| kappa_ema_alpha | **0.7** (map3) / **0.5** (map1) | smoothing |
| kappa_speed_factor | 2.0 | `v_ref = target_v / (1+factor·|κ|)` |
| v_min_curve | 0.3 m/s | v_ref lower bound |
| alat_max / along_max | 4.0 m/s² | soft constraints |
| solver_build_dir | `nmpc_build_6state` | shared map1/2/3/4 |

### Cost weights (hardcoded trong `acados_settings.py:105-109`)

```
Q  = diag([1e-1, 5e1, 1e1, 1e-1, 1e-3, 5e-3])   # s, n, α, v, D, δ
R  = diag([1e-3, 5e-3])                         # derD, derDelta
Qe = diag([5e-1, 1e2, 2e1, 1e-1, 5e-3, 2e-3])   # terminal
```

`ocp.cost.W = unscale * blkdiag(Q, R)`, `unscale = N/Tf = 20`.

---

## 7. Deadlines

| Milestone | Date | Status |
|-----------|------|--------|
| Rough demo / draft | 2026-04-18 | ❌ quá hạn |
| Sim closed (baseline bag 11) | 2026-04-19 | ✅ DONE |
| Hardware lane following 0.3 m/s | 2026-04-23 | 🔄 NOW |
| Thesis draft (chương 2-5) | 2026-04-24 | 🔄 song song |
| Final submission | 2026-04-25 | ⚠️ còn 6 ngày |

Xem [documents/plan/02_phase_future.md](documents/plan/02_phase_future.md) cho
priority order 6 ngày còn lại.

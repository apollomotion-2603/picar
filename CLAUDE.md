# CLAUDE.md — RC Car Autonomous Driving
apollomotion (Phi) & Nhã | ĐH GTVT TP.HCM | Updated 2026-04-19

Vai trò file này: entry point cho AI — project identity, roadmap, conventions.
Chi tiết kỹ thuật (params, commands, issues) ở `PROJECT_STATE.md` và `documents/state/`.
Chi tiết phase ở `documents/plan/`.

---

## 1. PROJECT IDENTITY

**Mục tiêu:** Xe RC 1/16 scale tự hành — luận văn tốt nghiệp, hướng tới sản phẩm
thương mại cho doanh nghiệp, đại học, sinh viên mới.

**Phân công:**

| Phi (apollomotion) | Nhã |
|---|---|
| Perception pipeline | NMPC model + solver |
| EKF state estimation | Velocity / throttle control |
| SLAM + Localization | Nucleo firmware |
| Nav2 integration | Constraint tuning |
| GUI Dashboard | Hardware motor control |
| Luận văn (chính) | Đóng góp chương 3, 4 |

**Nguyên tắc thiết kế:**
- *Build once, reuse forever* — hardware modular (JST), software parameter-driven.
- *Make it work first, optimize later.*
- Thay track → chỉnh `perception.yaml` (BEV corners).
- Thay xe → chỉnh `robot.yaml`.
- Sim và hardware phát triển song song.

**Deadlines (CRITICAL — quá hạn rough, sắp đến final):**
- ~~April 18, 2026 — Rough product~~ → **QUÁ HẠN** (hôm nay 2026-04-19).
- **April 25, 2026 — Final submission** (còn 6 ngày).

---

## 2. ROADMAP (12 tuần)

| Tuần | Phase | Mục tiêu | Status |
|------|-------|-----------|--------|
| 1–4 | Phase 1 — Simulation | Perception + NMPC + EKF trong Gazebo | ✅ DONE |
| 5–6 | Phase 1.5a — Sim tracks (map 1 + map 3) | 2 map sim đạt yêu cầu | ✅ DONE |
| 7–8 | Phase 1.5b — Hardware lane following | Xe bám lane track thật 0.3 m/s | 🔄 NOW |
| — | Thesis writing (song song) | Chương 2-5 | 🔄 NOW |
| 9–10 | Phase 2 — SLAM | Visual odometry + lane-based map | 📅 scope giảm |
| 11 | Phase 3 — Navigation | Nav2 + obstacle avoidance | 📅 scope giảm |
| 12 | Phase 4 — GUI + Demo | Foxglove dashboard + luận văn | 📅 |

Chi tiết → [documents/plan/01_phase_current.md](documents/plan/01_phase_current.md),
[02_phase_future.md](documents/plan/02_phase_future.md).

---

## 3. CURRENT FOCUS — Phase 1.5b (Hardware) + Thesis

**Quyết định 2026-04-19:** Sim đủ tốt cho demo → **chuyển sang deploy HW +
viết thesis**. Không tune thêm sim, ROI thấp so với deadline 6 ngày.

### Sim — closed (baseline cho thesis)
- [x] Map 1 `lane_track.sdf` (9×2m oval) — bám lane mượt ✅
- [x] Map 3 `lane_change.sdf` (8×2m DLC) — đổi làn thành công ✅
  - Baseline: `bag_map_test_11` (`|n|max=72mm`, `|κ|max=0.76`, δ max=15°).
  - Fix cuối cùng: per-stage EMA với arc-length uniform `s_j = j·s_max/N`
    (v-independent) — giữ preview 1s + stable.
- [~] Map 2 `track_test.sdf` — drop scope, không đủ thời gian.
- [~] Map 4 `obstacle_track.sdf` — drop scope, cần Object Detection.

### Hardware deploy (blockers theo thứ tự)
- [ ] **#1 Pi Camera 3 CFE driver** (`Unable to acquire CFE instance`) —
  thử RPi OS 64-bit + Docker; fallback USB webcam nếu vẫn fail 1 ngày.
- [ ] **#2 Camera intrinsic calibration** (checkerboard).
- [ ] **#3 BEV recalibration** cho camera thật → `perception.yaml`
  (+ sync `visualizer_node.py:17-18` hardcoded).
- [ ] **#4 Nucleo serial parser** BNO055 → `/imu/data`.
- [ ] **#5 Test lane straight 0.3 m/s** trên track giấy A0 + băng keo.

### Thesis (song song — viết mỗi ngày)
- Chương 2 Perception — dựa `documents/technical_docs/02_PERCEPTION_AND_EKF.md`.
- Chương 3 EKF — ibid.
- Chương 4 NMPC (Nhã chính) — `03_NMPC_CONTROL_LOGIC.md` + bag 11 data.
- Chương 5 Thí nghiệm — bag 11 (sim DLC) + video HW (khi có).

---

## 4. HARDWARE OVERVIEW

```
LiPo 3S
├── 5V → Raspberry Pi 5 (phibui@picar, 192.168.1.23)
│        ├── Pi Camera 3 Wide (IMX708, CSI) → /camera/image_raw
│        └── USB → Nucleo F411RE (/dev/ttyACM0)
│                 ├── BNO055 IMU (I2C) → /imu/data
│                 ├── Servo PWM → Steering
│                 └── ESC PWM → Drive motor
└── 6V → Servo backup

Laptop (apollomotion, 192.168.1.17): Ubuntu 22.04, ROS2 Humble,
Ignition Gazebo 6 Fortress, acados v0.3.5 @ ~/acados/, ROS_DOMAIN_ID=42.
```

**Xe specs:** m=2.5kg · lf=lr=0.11m · wheel_r=0.04m · δ_max=0.6109rad ·
lane_width=0.5m · v_max sim=1.5 m/s · v_max HW phase 1=0.3 m/s.

Chi tiết → [documents/plan/03_hardware.md](documents/plan/03_hardware.md).

---

## 5. WORKING CONVENTIONS

### ROS2
- `source install/setup.bash` trước mọi lệnh ROS2.
- `ROS_DOMAIN_ID=42` cho mọi terminal (laptop + Pi).
- Build conflict: `rm -rf build/ install/` rồi build lại.
- **Đổi `nmpc.yaml` hoặc `bicycle_model.py` → PHẢI** `rm -rf nmpc_build_6state/`
  trước khi chạy lại (solver cache cũ).

### Code style
- Nodes phải có timeout + fallback an toàn (publish 0/0 khi mất sensor).
- Không hardcode params — externalize ra yaml.
- `colcon build --symlink-install` để edit Python nodes không cần rebuild.
- Log NMPC format: `n=Xmm α=X° v=X(src) D=X δ=X° κ₀=X vref₀=X v→X t=Xms st=X`.

### File creation
- File dài (>20 dòng): `python3 << 'PYEOF' ... PYEOF`.
- File ngắn: `cat > file.py << 'EOF'`.

### AI workflow
- **Claude Web:** brainstorm, debug, review, visual (Gazebo screenshot).
- **Claude Code:** edit file, refactor, create file — chỉ khi cần filesystem.
- Không đọc quá nhiều file một lúc — tốn token.

### Ignition Fortress protobuf (gotcha)
- Dùng `ign service`, không phải `gz service`.
- Msg type `ignition.msgs.*`, không phải `gz.msgs.*`.
- Format text: `position: {x: X y: Y z: Z}` — **phải có `:`** trước nested.

---

## 6. RISK & FALLBACK

| Rủi ro | P | Fallback |
|--------|---|----------|
| Pi Camera 3 driver | Cao | USB camera hoặc picamera2 |
| Visual SLAM drift HW | TB | Pre-built map + AMCL |
| Nav2+NMPC integrate | TB | Nav2 DWB controller |
| Luận văn trễ deadline | Cao | Viết song song từ tuần 3 |
| Timeline cực tight (6 ngày) | Cao | Ưu tiên sim demo + báo cáo; HW scope giảm |

---

## 7. REFERENCES

- **Kloeser 2020** — *NMPC for Racing Using a Singularity-Free Path-Parametric
  Model.* Base cho 6-state bicycle, acados RTI+HPIPM.
- **acados v0.3.5** — `~/acados/`. WARNING *Gauss-Newton + EXTERNAL* bình thường.
- **tera_renderer v0.2.0** — build từ Rust source (required by acados).
- **Ignition Gazebo 6 Fortress** — `ign` CLI, `ignition.msgs.*`.

---

## 8. POINTERS

- [PROJECT_STATE.md](PROJECT_STATE.md) — technical index.
- [documents/plan/](documents/plan/) — phase plans.
- [documents/state/](documents/state/) — current state chi tiết.
- [documents/technical_docs/](documents/technical_docs/) — thesis-style writeup.

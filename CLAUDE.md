# CLAUDE.md — RC Car Autonomous Driving
# apollomotion (Phi) & Nhã | ĐH GTVT TP.HCM | 2026
# Vai trò file này: Plan + Context + Conventions cho AI
# Xem PROJECT_STATE.md cho technical details (code, commands, params)

---

## 1. PROJECT IDENTITY

**Mục tiêu:** Xe RC tự hành 1/16 scale — luận văn tốt nghiệp, hướng tới sản phẩm thương mại
cho doanh nghiệp, đại học, sinh viên mới.

**Phân công:**
| Phi (apollomotion) | Nhã |
|---|---|
| Perception pipeline | NMPC optimization |
| EKF state estimation | Velocity control |
| SLAM + Localization | Nucleo firmware |
| Nav2 integration | Constraint tuning |
| GUI Dashboard | Hardware motor control |
| Luận văn (chính) | Đóng góp chương 3, 4 |

**Nguyên tắc thiết kế xuyên suốt:**
- **"Build once, reuse forever"** — hardware modular (JST connector), software parameter-driven
- **"Make it work first, optimize later"**
- Thay track/môi trường → chỉ chỉnh `perception.yaml` (BEV corners)
- Thay xe → chỉnh `robot.yaml`
- Sim và hardware phát triển song song

**Deadlines:**
- **April 18, 2026** — Rough product: sim + hardware + draft report
- **April 25, 2026** — Final submission

---

## 2. ROADMAP TỔNG THỂ (12 tuần)

| Tuần | Phase | Mục tiêu | Status |
|------|-------|-----------|--------|
| 1–2 | **Phase 1:** Simulation | Perception + NMPC + EKF trong Gazebo | ✅ XONG |
| 3–4 | **Phase 1.5a:** Sim tracks + HW prep | 3 track sim + camera Pi deploy | 🔄 ĐANG LÀM |
| 5–6 | **Phase 1.5b:** Hardware lane following | Xe bám lane track thật 0.3 m/s | 📅 KẾ HOẠCH |
| 7–8 | **Phase 2:** SLAM | Visual odometry + lane-based map | 📅 KẾ HOẠCH |
| 9–10 | **Phase 3:** Navigation | Nav2 + obstacle avoidance | 📅 KẾ HOẠCH |
| 11 | **Phase 4:** GUI | Foxglove dashboard | 📅 KẾ HOẠCH |
| 12 | **Luận văn + Demo** | Video + demo live | 📅 KẾ HOẠCH |

---

## 3. CURRENT FOCUS — Phase 1.5a (Tuần 3–4)

**Mục tiêu tuần này:** Hoàn thiện simulation + chuẩn bị hardware deploy

### Sim tracks checklist
- [x] Track 1: Oval đơn giản (`lane_track.sdf`, 9×2m) — xe bám lane mượt ✅
- [ ] Track 2: `track_test.sdf` (6×6m, chicane + S-curve) — test full pipeline
- [x] Track 3: Chuyển làn kép (`lane_change.sdf`, 8×2m) — đã test thành công NMPC ✅
- [ ] Track 4: Track tĩnh vật cản (`obstacle_track.sdf`) — thiết kế xong SDF, chờ test chướng ngại vật

### Hardware prep checklist
- [ ] Giải quyết Pi Camera 3 driver (`Unable to acquire CFE instance`)
- [ ] Calibrate camera intrinsics (checkerboard)
- [ ] Test stream `/camera/image_raw` từ Pi về laptop

### Blockers hiện tại
- Pi Camera 3 CFE driver conflict trên kernel 6.8 — chưa resolve
- Track 2 spawn position cần verify sau khi test

---

## 4. PHASE 1 — Simulation ✅ XONG

### Pipeline hoàn chỉnh (closed loop)
```
Camera (30Hz) → perception_node → /perception/lane_state
                                         ↓
IMU (100Hz) ──→ ekf_node ────────→ /ekf/vehicle_state
Joint States ──→    ↑                    ↓
                                   mpc_node (NMPC)
                                    ↓           ↓
                            /steering_angle  /velocity
                                         ↓
                              vehicle_controller (C++)
                                         ↓
                                   Gazebo physics
```

### Kết quả đạt được
- [x] Perception: BEV → Sliding Window → Cubic poly + arc-length @ 30Hz
- [x] EKF: Camera + IMU + Wheel encoder, `ekf_healthy=true` @ 100Hz
- [x] NMPC Kloeser 2020: acados RTI+HPIPM, solve time 1–5ms @ 30Hz
- [x] Curvature-based adaptive speed: `v_ref = V_MAX / (1 + 8·|κ|)`
- [x] EKF velocity feedback thay first-order model
- [x] Reset node: S/X/Q keyboard + `/reset_car` service + auto-reset khi lane lost
- [x] Full pipeline launch: `sim_full_launch.py` (1 lệnh, 2 terminal)
- [x] 4-panel debug visualizer: Raw+ROI / BEV / Threshold / Lanes

### NMPC key params
| Tham số | Giá trị | Ghi chú |
|---------|---------|---------|
| V_MAX | 2.0 m/s | Sim; hardware giai đoạn 1: 0.3 m/s |
| N (horizon) | 30 steps | Tf=1.0s, dt=33ms |
| w_n (lateral) | 200.0 | Penalize lane offset |
| w_alpha (heading) | 100.0 | Penalize heading error |
| kappa_factor | 8.0 | Speed reduction tại cua |
| Solve time | 1–5ms | Margin 28ms so với 33ms period ✅ |

---

## 5. PHASE 1.5a — Sim Tracks + HW Prep 🔄 ĐANG LÀM

### Track 2 — `track_test.sdf` (6×6m, chicane + S-curve)
- Texture: `track_test.png` (2048×2048px)
- Spawn đúng: `x=0.0, y=-2.666, Y=0.0` (bottom straight centerline)
- Launch: `ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=2`
- Reset node: `ros2 run ekf_pkg reset_node --ros-args -p map_id:=2`
- **TODO:** Test full pipeline, tune nếu cần

### Track 3 — `lane_change.sdf` (Double lane change) ✅XONG
- Đã test thành công xe có khả năng tự bám và chuyển làn.
- Tọa độ spawn và reset: `x=0.54, y=0.75, Y=0.0` (giữa làn phải).

### Track 4 — `obstacle_track.sdf` (Có vật cản tĩnh)
- Đã tạo xong SDF từ map 1 và chèn các static box model.
- Chờ tích hợp Object Detection để NMPC tự né.

### Hardware prep
- **Pi Camera 3:** Resolve CFE issue → thử `media-ctl --print-topology`
- **Calibration:** Checkerboard intrinsics → update `perception.yaml` BEV SRC/DST
- **Nucleo serial parser:** Viết node parse BNO055 → `/imu/data`

---

## 6. PHASE 1.5b — Hardware Lane Following 📅 KẾ HOẠCH

**Mục tiêu:** Xe bám lane track thật ở 0.3 m/s. Chỉ điều khiển góc lái — tốc độ cố định.

### Pipeline deploy (không cần EKF giai đoạn đầu)
```
Pi Camera (CSI)
  → perception_node (Pi, Docker ROS2)
  → /camera/image_raw + /perception/lane_state
  → [ROS2 network, ROS_DOMAIN_ID=42]
  → mpc_node (laptop)
  → /steering_angle, /velocity
  → [ROS2 network]
  → Nucleo F411RE (Pi, USB /dev/ttyACM0)
  → Servo (steering) + ESC (drive)
```

### Track thật — thông số
| Tham số | Giá trị |
|---------|---------|
| Nền | Giấy trắng A0 (4–8 tờ ghép) |
| Lane marking | Băng keo đen, rộng 2–3cm |
| Lane width | 50cm (2 dải song song) |
| Kích thước tối đa | ~3.4m × 2.4m (8 tờ A0) |

### Hardware checklist
- [ ] Giải quyết Pi Camera 3 driver
- [ ] Calibrate camera intrinsics (checkerboard)
- [ ] Recalibrate BEV homography SRC/DST → update `perception.yaml`
- [ ] Test perception node trên Pi, stream `/camera/image_raw` về laptop
- [ ] Tune AdaptiveThreshold cho băng keo đen trên nền trắng
- [ ] Verify NMPC nhận `/perception/lane_state` từ Pi qua network
- [ ] Test 0.3 m/s trên đường thẳng
- [ ] Test cua nhẹ sau khi đường thẳng ổn định
- [ ] Record video milestone

---

## 7. PHASE 2 — SLAM & Localization 📅 Tuần 7–8

**Mục tiêu:** Xe build map realtime từ camera, biết vị trí của mình trên map.

**Approach:**
- Visual Odometry: estimate motion giữa các frame liên tiếp
- Lane Graph Map: graph của lane centerline (nodes + edges), lưu JSON
- Particle Filter Localization: localize xe trên lane graph

**Lý do:** Môi trường băng keo đen / nền trắng = contrast cao, lane geometry rõ ràng
→ không cần feature-based SLAM phức tạp.

### Sim checklist
- [ ] Ground truth odometry từ Gazebo để validate VO
- [ ] Implement visual odometry node
- [ ] Build lane graph từ perception output
- [ ] Implement particle filter localization
- [ ] Visualize map + xe position trong RViz/Foxglove

### Hardware checklist
- [ ] Test visual odometry trên Pi Camera 3 thật
- [ ] Build map track thật 1 và 2
- [ ] Validate localization accuracy
- [ ] Record video: xe build map realtime

---

## 8. PHASE 3 — Navigation & Obstacle Avoidance 📅 Tuần 9–10

**Mục tiêu:** Xe tự navigate từ A → B, tránh vật cản. Full autonomy.

### Stack kỹ thuật
- **ROS2 Nav2:** `map_server` + `amcl` + `bt_navigator`
- **NMPC làm local controller** (thay DWB mặc định của Nav2)
- **Obstacle detection:** camera-based (băng keo hoặc object đặt trên track)

### Interface Nav2 → NMPC
```
Nav2 → /cmd_vel (linear.x, angular.z)
     → convert → NMPC reference trajectory
     → NMPC → /steering_angle, /velocity
```

### Checklist
- [ ] Thêm obstacle model vào `track_test.sdf`
- [ ] Test Nav2 với lane map từ Phase 2
- [ ] Integrate NMPC làm Nav2 local controller
- [ ] Test obstacle avoidance sim + hardware
- [ ] Record video: xe tự navigate + tránh vật cản

---

## 9. PHASE 4 — GUI Dashboard 📅 Tuần 11

**Mục tiêu:** Dashboard cho non-technical audience — không cần hiểu ROS2 để vận hành xe.

### Stack
- **Primary:** Foxglove Studio + custom panels
- **Alternative:** `rosbridge_suite` + React web app (chạy trên browser bất kỳ)

### Features
- Camera feed realtime
- Map visualization + vị trí xe
- Vehicle status: tốc độ, góc lái, mode (manual/auto)
- Mission control: set goal, start/stop, reset xe
- Alert system: lane lost, obstacle detected, EKF unhealthy
- Parameter tuning panel: V_MAX, weights (nice-to-have)

---

## 10. HARDWARE ARCHITECTURE

```
LiPo 3S
├── 5V reg ──→ Raspberry Pi 5 (phibui@picar, 192.168.1.23)
│             ├── Pi Camera 3 Wide (IMX708, CSI) → /camera/image_raw
│             └── USB → Nucleo F411RE (/dev/ttyACM0)
│                       ├── BNO055 IMU (I2C) → /imu/data
│                       ├── Servo PWM → Steering
│                       └── ESC PWM → Drive motor
└── 6V reg ──→ Servo (backup power)

Laptop (apollomotion, 192.168.1.17):
├── ROS2 Jazzy native (Ubuntu 24.04)
├── Gazebo Harmonic v8.10
├── acados v0.3.5 @ ~/acados/
└── ROS_DOMAIN_ID=42
```

### Hardware specs xe RC 1/16
| Parameter | Value |
|-----------|-------|
| m | 2.5 kg |
| lf = lr | 0.11 m |
| L (wheelbase) | 0.22 m |
| wheel_radius | 0.04 m |
| max_steering | 0.6109 rad (≈35°) |
| lane_width | 0.50 m |
| V_max sim | 2.0 m/s |
| V_max hardware (giai đoạn 1) | 0.3 m/s |

---

## 11. RISK & FALLBACK

| Rủi ro | Xác suất | Fallback |
|--------|----------|----------|
| Pi Camera 3 driver không ổn định | Cao | USB camera hoặc picamera2 |
| Visual SLAM drift trên track thật | Trung bình | Pre-built map + AMCL |
| Nav2 + NMPC integration phức tạp | Trung bình | Nav2 dùng DWB controller |
| Luận văn không kịp deadline | Cao | Viết song song từ tuần 3 |
| Hardware mechanical failure | Thấp | Spare parts, JST connector dễ swap |
| Timeline quá tight | Cao | Phase 4 GUI đơn giản hóa nếu cần |

---

## 12. WORKING CONVENTIONS

### File creation
- Dùng `python3 << 'PYEOF' ... PYEOF` để ghi file (không dùng heredoc `cat << EOF` cho
  file dài vì escape issue)
- Với file ngắn < 20 dòng: `cat > file.py << 'EOF'`

### ROS2
- Luôn `source install/setup.bash` trước khi chạy lệnh ROS2
- `ROS_DOMAIN_ID=42` cho tất cả terminals (laptop + Pi)
- Build sạch khi gặp CMakeCache conflict: `rm -rf build/ install/` rồi rebuild
- Xóa NMPC solver cache khi thay đổi nmpc.yaml: `rm -rf nmpc_build/`

### Code style
- Tất cả ROS2 nodes phải có timeout check và fallback
- Log format NMPC: `n=Xmm α=X° v=Xm/s(EKF/model) δ=X° κ=X t=Xms`
- Không hardcode params trong code — externalize ra yaml
- `--symlink-install` để edit source trực tiếp không cần rebuild Python nodes

### AI workflow
- **Claude Web** (claude.ai): brainstorm, debug logic, review, visual tasks (Gazebo screenshot)
- **Claude Code**: edit file, refactor, tạo file mới — chỉ dùng khi cần truy cập filesystem
- Không để Claude Code đọc quá nhiều file một lúc — tốn token không cần thiết

---

## 13. REFERENCES

- **Kloeser 2020:** "NMPC for Racing Using a Singularity-Free Path-Parametric Model"
  - State: `x=[s,n,α,v]`, Control: `u=[delta,v_ref]`
  - acados RTI + HPIPM solver, validated 1:43 scale RC car @ 50Hz

- **acados v0.3.5:** `~/acados/`, WARNING "Gauss-Newton + EXTERNAL" là bình thường
- **tera_renderer v0.2.0:** build từ source Rust (required by acados)
- **ROS2 Jazzy + Gazebo Harmonic v8.10:** không dùng Gazebo Classic

---

> **Technical details (code, commands, params, topics, known issues):**
> → Xem `PROJECT_STATE.md`

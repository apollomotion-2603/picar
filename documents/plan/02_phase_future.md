# 02. Phase 2 / 3 / 4 — Future work

Updated 2026-04-19. Deadline final: 2026-04-25 (6 ngày) → các phase dưới đây
khả năng giảm scope cho kịp luận văn.

---

## Phase 2 — SLAM & Localization (📅 Tuần 7–8)

**Mục tiêu:** Xe build map realtime từ camera, biết vị trí trên map.

### Approach
- **Visual Odometry** — estimate motion giữa frame liên tiếp.
- **Lane Graph Map** — graph centerline (nodes + edges), lưu JSON.
- **Particle Filter Localization** — localize xe trên lane graph.

**Lý do đơn giản hoá:** môi trường băng keo đen / nền trắng = contrast cao,
lane geometry rõ → không cần feature-based SLAM phức tạp.

### Sim checklist
- [ ] Ground truth odometry từ Gazebo để validate VO.
- [ ] Implement visual odometry node.
- [ ] Build lane graph từ perception output.
- [ ] Implement particle filter localization.
- [ ] Visualize map + position trong RViz/Foxglove.

### Hardware checklist
- [ ] Test VO trên Pi Camera 3 thật.
- [ ] Build map track 1 và 2 thật.
- [ ] Validate localization accuracy.
- [ ] Record video: xe build map realtime.

---

## Phase 3 — Navigation & Obstacle Avoidance (📅 Tuần 9–10)

**Mục tiêu:** Xe tự navigate A → B, tránh vật cản. Full autonomy.

### Stack
- **ROS2 Nav2:** `map_server` + `amcl` + `bt_navigator`.
- **NMPC làm local controller** (thay DWB mặc định của Nav2).
- **Obstacle detection:** camera-based (băng keo hoặc object trên track).

### Interface Nav2 → NMPC

```
Nav2 → /cmd_vel (linear.x, angular.z)
     → convert → NMPC reference trajectory
     → NMPC → /steering_angle, /velocity
```

### Checklist
- [ ] Tích hợp static obstacles vào `obstacle_track.sdf` (Map 4).
- [ ] Test Nav2 với lane map từ Phase 2.
- [ ] Integrate NMPC làm Nav2 local controller.
- [ ] Test obstacle avoidance sim + hardware.
- [ ] Record video: xe tự navigate + tránh vật cản.

---

## Phase 4 — GUI Dashboard (📅 Tuần 11)

**Mục tiêu:** Dashboard cho non-technical audience — không cần hiểu ROS2
vẫn vận hành được xe.

### Stack
- **Primary:** Foxglove Studio + custom panels.
- **Alternative:** `rosbridge_suite` + React web app (browser bất kỳ).

### Features
- Camera feed realtime.
- Map visualization + vị trí xe.
- Vehicle status: tốc độ, góc lái, mode (manual/auto).
- Mission control: set goal, start/stop, reset xe.
- Alert: lane lost, obstacle detected, EKF unhealthy.
- Parameter tuning panel: `v_max`, weights (nice-to-have).

---

## Phase 5 — Luận văn + Demo (📅 Tuần 12)

**Deliverables:**
- Luận văn hoàn chỉnh (viết song song từ tuần 3).
- Video demo sim + hardware.
- Live demo (optional, tùy tình trạng HW).

### Luận văn scope (chương chính)
1. Giới thiệu — animus thiết kế, phân công.
2. Perception pipeline — BEV + sliding window + cubic fit.
3. State estimation (EKF) — model + fusion.
4. NMPC control (Nhã, chính) — 6-state spatial bicycle + acados.
5. Thí nghiệm — sim 4 map + hardware deploy + kết quả.
6. Kết luận — kết quả, hạn chế, hướng phát triển.

---

## Risk & Fallback (ưu tiên theo timeline còn lại)

| Rủi ro | P | Fallback |
|--------|---|----------|
| Pi Camera 3 driver không ổn định | Cao | USB camera, picamera2 |
| Visual SLAM drift track thật | TB | Pre-built map + AMCL |
| Nav2+NMPC integrate phức tạp | TB | Dùng Nav2 DWB controller |
| Luận văn không kịp deadline | Cao | Ưu tiên viết, cắt Phase 4 GUI |
| HW mechanical failure | Thấp | Spare parts, JST connector swap |
| Timeline 6 ngày quá tight | Cao | Scope giảm: demo sim + HW stub + báo cáo |

**Priority order (6 ngày còn lại — cập nhật 2026-04-19):**
1. ✅ ~~Fix Map 3 DLC~~ — done bag 11 baseline.
2. ~~Test Map 2 chicane~~ — **drop scope**, không đủ thời gian.
3. **Hardware deploy cơ bản** (lane straight 0.3 m/s) — blocker order ở
   [01_phase_current.md](01_phase_current.md).
4. **Viết thesis song song mỗi ngày** — chương 4 NMPC (Nhã chính),
   chương 5 thí nghiệm (bag 11 + video HW khi có).
5. Phase 2/3/4 **drop scope** — không kịp deadline 2026-04-25.
6. Nếu HW fail → demo pure sim + báo cáo kỹ chương 5 sim results.

---

## Pointers

- Phase hiện tại: [01_phase_current.md](01_phase_current.md).
- Hardware specs: [03_hardware.md](03_hardware.md).
- Technical state: [../state/](../state/).

# 01. Phase 1.5b (NOW) — Hardware Deploy + Thesis

Updated 2026-04-19. Phase 1 + 1.5a (Sim) đã xong — xem
[../state/05_status_issues.md](../state/05_status_issues.md).

---

## Phase 1.5a — Sim tracks (✅ DONE 2026-04-19)

**Kết quả:** Sim đạt yêu cầu cho demo luận văn. Đóng phase, không tune thêm.

### Track status (closed)

| Map | File | Status | Baseline |
|-----|------|--------|----------|
| 1 | `lane_track.sdf` oval | ✅ bám lane mượt | `nmpc_map1.yaml` (`kappa_max=2.0`) |
| 2 | `track_test.sdf` chicane | ~ **drop scope** | thiếu thời gian |
| 3 | `lane_change.sdf` DLC | ✅ đổi làn thành công | `bag_map_test_11` |
| 4 | `obstacle_track.sdf` | ~ **drop scope** | cần Object Detection |

### Map 3 DLC — final baseline (bag 11)
- `|n|max=72mm` · `|κ|max=0.76` · δmax=14.8° · không saturate/stall.
- Early `ey=+1.5±3.7mm` (bias cua đầu ~0).
- Config: `nmpc.yaml` (`kappa_max=0.8`, `kappa_ema_alpha=0.7`).
- Fix cuối: per-stage EMA với `s_j = j·s_max/N` (arc-length uniform,
  v-independent) — giữ preview 1s và EMA hợp lệ.
- Remaining minor (accept): late-steer ~0.3s đoạn cuối DLC — HW 0.3 m/s
  sẽ không còn.

---

## Phase 1.5b — Hardware Deploy (🔄 NOW)

---

## Phase 1.5b — Hardware Lane Following (chi tiết)

**Mục tiêu:** Xe bám lane track thật ở 0.3 m/s. Chỉ điều khiển góc lái —
tốc độ cố định. Không cần EKF giai đoạn đầu.

### Blocker order (ưu tiên từ trên xuống)
1. **Pi Camera 3 CFE driver** — `Unable to acquire CFE instance`. Thử RPi
   OS 64-bit + Docker ROS2 Humble. Fallback USB webcam nếu fail 1 ngày.
2. **Camera intrinsic calibration** (checkerboard) → `camera_calibration`.
3. **BEV recalibration** cho camera thật → `perception.yaml` + sync
   `visualizer_node.py:17-18` (hardcoded).
4. **AdaptiveThreshold tune** cho băng keo đen / giấy trắng.
5. **Nucleo serial parser** BNO055 → `/dev/ttyACM0` → `/imu/data`.
6. **Test straight 0.3 m/s** → **test cua nhẹ** → record video.

### Pipeline deploy

```
Pi Camera 3 (CSI)
  → perception_node (Pi, Docker ROS2)
  → /camera/image_raw + /perception/lane_state
  → [ROS2 network, DOMAIN_ID=42]
  → mpc_node (laptop)
  → /steering_angle, /velocity
  → [ROS2 network]
  → Nucleo F411RE (Pi, /dev/ttyACM0)
  → Servo (steering) + ESC (drive)
```

### Track thật — thông số

| Tham số | Giá trị |
|---------|---------|
| Nền | Giấy trắng A0 (4–8 tờ ghép) |
| Lane marking | Băng keo đen, rộng 2–3cm |
| Lane width | 50cm (2 dải song song) |
| Kích thước tối đa | ~3.4m × 2.4m (8 tờ A0) |

### Checklist

- [ ] Resolve Pi Camera 3 driver.
- [ ] Calibrate camera intrinsics.
- [ ] Recalibrate BEV homography SRC/DST → update `perception.yaml`.
- [ ] Tune AdaptiveThreshold cho băng keo đen / giấy trắng.
- [ ] Test `perception_node` trên Pi, stream ảnh về laptop.
- [ ] Verify `mpc_node` nhận lane_state qua ROS2 network.
- [ ] Test 0.3 m/s đường thẳng.
- [ ] Test cua nhẹ sau khi đường thẳng ổn định.
- [ ] Record video milestone.

### BEV recalibration procedure

1. Print checkerboard, đặt phẳng trong FOV camera.
2. Run `camera_calibration` → camera intrinsics.
3. Xác định 4 góc lane rectangle trong ảnh → `bev_src`.
4. Đo kích thước vật lý → `bev_dst` khớp metric scale.
5. Update `perception_pkg/config/perception.yaml` `bev_src`/`bev_dst`.
6. Update `visualizer_node.py:17-18` (SRC hardcoded — nhớ sync!).

### Hardware-specific notes

- EKF có thể disable giai đoạn đầu — NMPC tự fallback sang first-order `v`.
- Pi Camera 3 Wide (IMX708): known issue CFE instance trên Ubuntu Server 24.04
  → dùng RPi OS 64-bit + Docker ROS2 Humble.
- BNO055 serial parser: chưa viết (Phase 1.5b TODO).
- Docker volume trên Pi: `~/ros2_ws:/ros2_ws`.

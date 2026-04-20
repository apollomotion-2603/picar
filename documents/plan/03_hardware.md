# 03. Hardware Architecture & Specs

Updated 2026-04-19.

---

## Block diagram

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
├── Ubuntu 22.04 LTS
├── ROS2 Humble (native)
├── Ignition Gazebo 6 / Fortress (`ign` CLI, không phải `gz`)
├── acados v0.3.5 @ ~/acados/
└── ROS_DOMAIN_ID=42
```

---

## Xe RC 1/16 — physical specs

| Parameter | Value | Note |
|-----------|-------|------|
| Mass (m) | 2.5 kg | body + 4 wheels |
| lf = lr | 0.11 m | symmetric |
| Wheelbase (L) | 0.22 m | lf + lr |
| Wheel radius | 0.04 m | |
| Body (L×W×H) | 0.30 × 0.18 × 0.05 m | |
| Max steering | 0.6109 rad | ≈ 35° |
| Lane width | 0.50 m | |
| v_max sim | 1.5 m/s | `v_max` trong nmpc.yaml |
| v_max HW phase 1 | 0.3 m/s | conservative deploy |

---

## Software stack

| Component | Version / Details |
|-----------|-------------------|
| OS laptop | Ubuntu 22.04 LTS |
| OS Pi | RPi OS 64-bit (Docker ROS2 Humble) |
| ROS2 | Humble (native laptop, Docker Pi) |
| Gazebo | Ignition 6 / Fortress (`ign` CLI) |
| acados | v0.3.5 @ `~/acados/` |
| tera_renderer | v0.2.0 (build từ Rust source) |
| `ACADOS_SOURCE_DIR` | `~/acados` (set trong `acados_settings.py:16`) |
| GPU laptop | NVIDIA Quadro T1000 4GB |
| `ROS_DOMAIN_ID` | 42 |

---

## Network topology (hardware)

```
Laptop (192.168.1.17) ←──WiFi──→ Pi (192.168.1.23)
     │                                │
     ├── mpc_node                     ├── perception_node
     ├── ekf_node                     ├── camera driver
     └── rqt / Foxglove               └── nucleo_bridge (TODO)

Shared bus: ROS_DOMAIN_ID=42 (DDS discovery qua multicast)
```

---

## Nucleo F411RE — serial protocol (planned)

Nucleo gửi BNO055 IMU data qua `/dev/ttyACM0` → Pi parse thành
`/imu/data` (sensor_msgs/Imu).

**Format proposed (chưa implement):**
```
$IMU,<timestamp_ms>,<qx>,<qy>,<qz>,<qw>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>\n
```

Nucleo nhận `/steering_angle` + `/velocity` qua serial → PWM servo/ESC.

---

## Hardware checklist state

### Đã có
- [x] Xe RC 1/16 lắp ráp xong.
- [x] LiPo 3S + regulators 5V/6V.
- [x] Raspberry Pi 5 flashed.
- [x] Pi Camera 3 Wide gắn lên xe.
- [x] Nucleo F411RE + BNO055 I2C.
- [x] Servo + ESC wired.

### Chưa xong
- [ ] Pi Camera 3 CFE driver stable.
- [ ] Camera intrinsics calibration.
- [ ] BEV homography recalibration cho camera thật.
- [ ] Nucleo firmware + serial parser node.
- [ ] End-to-end test: Pi perception → laptop NMPC → Nucleo → motor.

---

## Ignition Gazebo Fortress — gotchas

### Service teleport (dùng trong `reset_node`)

```bash
# ĐÚNG — Ignition Fortress
ign service -s /world/{world_name}/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "ackermann_steering_vehicle" position: {x: X y: Y z: 0.05} orientation: {x: 0 y: 0 z: QZ w: QW}'
```

**Protobuf text format gotchas:**
- PHẢI có `:` trước nested message: `position: {x: ...}` ✅
- KHÔNG dùng format mới `position { x: ... }` — Fortress parse fail ❌
- KHÔNG dùng dấu phẩy giữa fields trong `{}`.

### Sensors plugin
- KHÔNG add `ign-gazebo-sensors-system` vào world SDF — đã nằm trong
  vehicle xacro. Thêm twice gây segfault lúc load.

### IMU plugin
- Bảo đảm `ignition-gazebo-imu-system` plugin có trong `vehicle.xacro`
  — nếu không thì `/imu/data` không có data.

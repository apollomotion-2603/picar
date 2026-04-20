# 02. Launch, Build, Reset Commands

Updated 2026-04-19.

---

## Full pipeline (sim, 2 terminal)

```bash
export ROS_DOMAIN_ID=42
source ~/main/1_projects/1_autonomous_car_research/ros2_ws/install/setup.bash
```

**Terminal 1 — full sim** (Map 1 default; dùng `map:=2/3/4` để đổi):

```bash
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=3
```

**Terminal 2 — reset node** (cần TTY cho keyboard):

```bash
ros2 run ekf_pkg reset_node --ros-args -p map_id:=1
# s + Enter  → START xe
# x + Enter  → STOP xe
# q + Enter  → RESET xe về spawn point
```

**Startup sequence** (từ `sim_full_launch.py`):

| Delay | Action |
|-------|--------|
| 0s | Ignition Gazebo + spawn + robot_state_publisher + ros_gz_bridge + vehicle_controller |
| after spawn | joint_state_broadcaster activated |
| after JSB | forward_velocity_controller + forward_position_controller |
| +5s | `perception_node` |
| +6s | `ekf_node` |
| +6.5s | `visualizer_node` |
| +7s | `mpc_node` (build solver ~5s lần đầu, ~30s nếu rebuild 6-state) |
| +7.5s | `grid_viewer` |

`reset_node` **không còn trong launch** — chạy riêng terminal 2.

---

## Individual nodes (manual multi-terminal)

```bash
# T1 — Gazebo
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py \
  world:=$(ros2 pkg prefix gazebo_ackermann_steering_vehicle)/share/gazebo_ackermann_steering_vehicle/worlds/lane_track.sdf \
  x:=1.5 y:=1.0

# T2 — Perception
ros2 run perception_pkg perception_node --ros-args --params-file \
  src/perception_pkg/config/perception.yaml

# T3 — EKF
ros2 run ekf_pkg ekf_node

# T4 — NMPC (map 3; dùng nmpc_map1.yaml cho map 1)
ros2 run mpc_pkg mpc_node --ros-args --params-file \
  src/mpc_pkg/config/nmpc.yaml

# T5 — Reset node
ros2 run ekf_pkg reset_node --ros-args -p map_id:=2
```

---

## Reset / control commands

```bash
# Service call reset (khi reset_node chạy headless)
ros2 service call /reset_car std_srvs/srv/Trigger

# Debug visualizer
rqt_image_view /perception/debug_grid
```

---

## Build commands

```bash
cd ~/main/1_projects/1_autonomous_car_research/ros2_ws

# Build tất cả
colcon build --symlink-install

# Build từng package
colcon build --packages-select lane_msgs --symlink-install
colcon build --packages-select perception_pkg --symlink-install
colcon build --packages-select ekf_pkg --symlink-install
colcon build --packages-select mpc_pkg --symlink-install
colcon build --packages-select gazebo_ackermann_steering_vehicle --symlink-install

source install/setup.bash

# BẮT BUỘC khi thay đổi nmpc.yaml hoặc bicycle_model.py:
rm -rf nmpc_build_6state/

# Fix build conflicts
rm -rf build/lane_msgs install/lane_msgs
```

---

## Map configurations

| Map | Arg | World SDF | Spawn (x, y, Y) | Reset (x, y, Y) | Config |
|-----|-----|-----------|-----------------|-----------------|--------|
| 1 | `map:=1` | `lane_track.sdf` | (1.5, 1.0, 0) | (0.0, 1.0, 0) | `nmpc_map1.yaml` |
| 2 | `map:=2` | `track_test.sdf` | (0.0, -2.666, 0) | (0.0, -2.666, 0) | `nmpc.yaml` (tentative) |
| 3 | `map:=3` | `lane_change.sdf` | (0.54, 0.75, 0) | (0.54, 0.75, 0) | `nmpc.yaml` |
| 4 | `map:=4` | `obstacle_track.sdf` | (1.5, 1.0, 0) | (0.0, 1.0, 0) | `nmpc_map1.yaml` |

**Map 1 spawn ≠ reset:** intentional. Spawn tại track entry, reset tại lane
center origin (xe quay một vòng → về chỗ reset).

### Map descriptions

- **Map 1 — `lane_track.sdf`:** oval 9×2m, closed loop, no chicane.
  Lane texture `lane_track.png` (4096×910px).
- **Map 2 — `track_test.sdf`:** complex 6×6m, chicane + S-curve.
  Lane texture `track_test.png` (2048×2048px). Spawn Y=0.
- **Map 3 — `lane_change.sdf`:** Double Lane Change (DLC) 8×2m.
  Lane texture `lane_change.png`. Đã test NMPC đổi làn thành công.
- **Map 4 — `obstacle_track.sdf`:** dựa trên Map 1 + static box obstacles.
  Chờ Object Detection tích hợp.

---

## Bag recording & analysis

```bash
# Record bag (trong data/bags/)
cd data/bags
ros2 bag record -o bag_map_test_N \
  /perception/lane_state /ekf/vehicle_state \
  /steering_angle /velocity /car_enabled /tf

# Analyze (sau khi xong)
python3 data/bags/analyze_bag7.py data/bags/bag_map_test_N
```

Script `analyze_bag7.py` extract n, α, κ, v_cmd, δ_cmd, tf pose và in
stats per-section (early/middle/late) + detect oscillation qua δ sign flips.

---

## Teleport internal (dùng trong `reset_node`)

```bash
# ĐÚNG — Ignition Fortress:
ign service -s /world/{world_name}/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "ackermann_steering_vehicle" position: {x: X y: Y z: 0.05} orientation: {x: 0 y: 0 z: QZ w: QW}'

# SAI (parse fail):
#   position { x: ... }               ← thiếu `:`
#   --reqtype gz.msgs.Pose             ← Fortress dùng ignition.msgs.*
```

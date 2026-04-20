# 11 — Launch Files & World SDF

Package: `src/gazebo_ackermann_steering_vehicle/`

File chính:
- `launch/sim_full_launch.py` (154 dòng) — **entry point end-to-end sim**.
- `launch/vehicle.launch.py` (189 dòng) — launch Gazebo + xe không kèm nodes.
- `launch/track_test_launch.py` (112 dòng) — track_test riêng.
- `model/vehicle.xacro` (531 dòng) — URDF xe tham số hoá.
- `worlds/*.sdf` (4 files, 296 dòng tổng) — 4 track sim.

---

## 1. MỤC ĐÍCH

Launch system orchestrate: Gazebo + robot spawn + ros_gz_bridge +
ros2_control + perception + EKF + NMPC + visualizer + grid_viewer. 1 lệnh
= full stack sim.

Usage:
```bash
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=3
```

---

## 2. sim_full_launch.py — PHÂN TÍCH

### 2.1 Map selection

```python
MAP_CONFIGS = {
    "1": {"world": "lane_track.sdf",     "x": "0.0", "y": "1.0",    "Y": "0.0"},
    "2": {"world": "track_test.sdf",     "x": "0.0", "y": "-2.666", "Y": "0.0"},
    "3": {"world": "lane_change.sdf",    "x": "0.0", "y": "0.75",   "Y": "0.0"},
    "4": {"world": "obstacle_track.sdf", "x": "1.5", "y": "1.0",    "Y": "0.0"},
}
PERC_YAML_MAP = {"1":"perception_map1.yaml", "2":"perception_map1.yaml",
                 "3":"perception.yaml", "4":"perception_map1.yaml"}
NMPC_YAML_MAP = {"1":"nmpc_map1.yaml", "2":"nmpc_map1.yaml",
                 "3":"nmpc.yaml", "4":"nmpc_map1.yaml"}
```

- Map ID → world SDF + spawn pose (trùng `reset_node.MAP_CONFIGS`!).
- Map 3 (DLC) dùng bộ yaml riêng (`perception.yaml`, `nmpc.yaml`) vì track
  tính chất khác. Còn lại dùng `*_map1.yaml`.
- **Gotcha duplicate**: MAP_CONFIGS ở 2 chỗ (launch + reset_node). Đổi phải
  sync tay.

### 2.2 Argv parsing manually

```python
map_id_val = "1"
for arg in sys.argv:
    if arg.startswith("map:="):
        map_id_val = arg.split(":=")[1]
```

- **Không dùng `LaunchConfiguration` / `OpaqueFunction`** — parse thô từ
  `sys.argv` vì cần value **tại launch time** để chọn YAML path. ROS2
  launch substitution là lazy → không eval được trước khi build Node()
  arguments.
- Hậu quả: `DeclareLaunchArgument("map", default=...)` ở cuối file chỉ để
  hiện help text, không thực sự điều khiển.

### 2.3 `load_robot_description`

```python
def load_robot_description(xacro_path, params_path):
    with open(params_path) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    return xacro.process_file(xacro_path,
        mappings={k: str(v) for k, v in params.items()}).toxml()
```

- Đọc `parameters.yaml`, flatten params, truyền vào xacro `mappings`.
- `xacro` substitute `$(arg name)` → param value. Kết quả: URDF XML string
  truyền cho `robot_state_publisher` + `ros_gz_sim create`.
- Convention `/**` YAML namespace — match mọi node name.

### 2.4 Gazebo + spawn + bridge

```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros_gz_sim"),
                     "launch", "gz_sim.launch.py")),
    launch_arguments={"gz_args": f"-r -v 2 {world_path}",
                      "on_exit_shutdown": "true"}.items())
```
- `-r` auto-run (không pause ban đầu). `-v 2` verbosity.
- `on_exit_shutdown`: khi đóng Gazebo → kill toàn bộ launch.

```python
spawn = Node(
    package="ros_gz_sim", executable="create",
    arguments=["-name", "ackermann_steering_vehicle",
               "-string", robot_desc,
               "-x", cfg["x"], "-y", cfg["y"], "-z", "0.05",
               "-R","0.0", "-P","0.0", "-Y", cfg["Y"],
               "-allow_renaming", "false"])
```
- Tên `ackermann_steering_vehicle` PHẢI khớp với `reset_node` teleport.
- `-z=0.05` vừa đủ để xe nằm trên ground (wheel radius).
- `-allow_renaming=false` → fail nếu entity đã tồn tại (tránh clone ngầm).

```python
gz_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
    arguments=["--ros-args", "-p", f"config_file:={bridge_path}"])
```
Bridge Gazebo topics ↔ ROS2 qua `ros_gz_bridge.yaml` (define camera,
joint_states, velocity/steering commands).

### 2.5 ros2_control load sequence

```python
RegisterEventHandler(OnProcessExit(target_action=spawn,
                                   on_exit=[joint_state_ctrl]))
RegisterEventHandler(OnProcessExit(target_action=joint_state_ctrl,
                                   on_exit=[fwd_vel_ctrl, fwd_pos_ctrl]))
```

- Chain: spawn xong → load joint_state_broadcaster → load forward_velocity
  + forward_position controller.
- Dùng `ExecuteProcess` thay Node vì `ros2 control load_controller` là CLI,
  không phải node.

### 2.6 Timer delays cho nodes

```python
perception = TimerAction(period=5.0, actions=[Node(...)])
ekf        = TimerAction(period=6.0, actions=[Node(...)])
visualizer = TimerAction(period=6.5, actions=[Node(...)])
nmpc       = TimerAction(period=7.0, actions=[Node(...)])
grid_viewer = TimerAction(period=7.5, actions=[Node(...)])
```

- Gazebo + controllers cần ~4-5 s để ready. Start quá sớm → NMPC solver
  build xong trước khi có data → waste CPU + log noise.
- Thứ tự: perception (5s) → ekf (6s) → visualizer (6.5s) → nmpc (7s) →
  grid_viewer (7.5s). Ưu tiên node upstream trước.
- **Gotcha**: delay hardcoded — nếu máy chậm, Gazebo chưa ready lúc 5s,
  perception start mà không có camera → log timeout.

### 2.7 Params injection

```python
Node(package="perception_pkg", executable="perception_node",
     arguments=["--ros-args", "--params-file", perc_yaml])
```
- Truyền qua `--ros-args` thay vì `parameters=[...]` vì đã map theo `map_id`.
- EKF không truyền params-file (hardcoded trong node).

---

## 3. vehicle.launch.py (không-nodes)

Subset của sim_full_launch: chỉ có Gazebo + robot spawn + controllers +
ros_gz_bridge. Không có perception/EKF/NMPC. Dùng để debug Gazebo riêng
hoặc test manual joystick control.

Argument format dùng `LaunchConfiguration` chuẩn (world, x, y, z, R, P, Y)
— khác với `sim_full_launch` parse argv. Reason: file này không cần chọn
yaml theo map.

---

## 4. track_test_launch.py

Tương tự vehicle.launch.py nhưng preset `world=track_test.sdf`. Legacy —
có thể thay bằng `sim_full_launch.py map:=2`.

---

## 5. vehicle.xacro (531 dòng)

Tham số hoá toàn bộ xe:
- Body (length, width, height, density) → tính mass + inertia tự động.
- Wheels (radius, width, density) → tính wheel_mass, inertia trụ.
- Steering (max_steering_angle=0.6108 rad = 35°, max_angular_velocity,
  max_effort).
- Camera (height, pitch, FOV, FPS, image w/h).

Include Gazebo plugins:
- `gz_ros2_control` cho joint control (steering + drive).
- Camera sensor plugin publish `/camera/image_raw`.
- (Sau Bước C) IMU plugin publish `/imu/data`.

**Gotcha**: default `wheel_radius=0.3`, `body_length=2.0` — đó là kích
thước **xe to**. Khi sim RC car thật cần override qua `parameters.yaml`:
`wheel_radius=0.04`, scale tương ứng. Check file `config/parameters.yaml`.

---

## 6. World SDF files

Structure chung (xem `lane_track.sdf`):
- `<physics>`: `max_step_size=0.001` (1 ms), `real_time_factor=1.0`.
- 3 plugin core: `Physics`, `SceneBroadcaster`, `UserCommands`.
- `<light>`: directional sun.
- `<model>`: ground plane với PBR texture (albedo_map lane pattern).
- Collision plane infinite (không cần box cho đất).

**Per-map:**
| Map | World | Size | Lane type | Status |
|---|---|---|---|---|
| 1 | lane_track.sdf | 9×2m | Straight lane | ✅ done |
| 2 | track_test.sdf | ? | Chicane | drop scope |
| 3 | lane_change.sdf | 8×2m | DLC (double lane change) | ✅ done |
| 4 | obstacle_track.sdf | ? | With obstacles | drop scope |

Texture map nằm trong `media/materials/textures/` install path. **Gotcha**:
`albedo_map` hardcoded absolute path `/home/phibui2112/...` → **world SDF
không portable**. TODO: dùng `model://` relative path.

---

## 7. GOTCHAS TỔNG

- **MAP_CONFIGS duplicate** giữa launch + reset_node.
- **Hardcoded workspace path** `~/main/1_projects/…` trong launch dòng 48
  + world SDF albedo → không portable giữa máy.
- **Timer delays hardcoded** 5–7.5s. Máy chậm dễ fail.
- **Argv parsing `map:=`** không tích hợp launch substitution → khó
  compose với launch file khác (include từ ngoài).
- **`-z=0.05` spawn** — chỉnh theo wheel_radius. Với wheel thật 0.04 m,
  giá trị này OK. Khác wheel size phải chỉnh.
- **Gazebo Fortress + ros_gz_sim**: package name + API khớp với
  `gz-sim-*` plugins. Upgrade Gazebo Garden → phải đổi `gz-` prefix.
- **Visualizer + perception node share yaml** nhưng không đảm bảo cùng
  version — đổi yaml rồi launch không rebuild package có thể giữ cache cũ
  (thực ra Python load tại runtime nên OK, chỉ hardcode constants trong
  visualizer_node mới gây mismatch).
- **Grid viewer launch** yêu cầu có DISPLAY → chạy SSH không forward X
  sẽ crash.
- **Không có launch arg cho `use_sim_time`** — hardcoded `True` trong
  `robot_state_publisher`.

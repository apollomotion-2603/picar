# 05 — reset_node.py

File: `src/ekf_pkg/ekf_pkg/reset_node.py` (185 dòng)

---

## 1. MỤC ĐÍCH

- Control-panel node cho Gazebo sim: START / STOP / RESET xe thông qua
  keyboard hoặc ROS service `/reset_car` (std_srvs/Trigger).
- Teleport xe về spawn point (khác nhau theo `map_id`) bằng Ignition service
  `/world/<w>/set_pose`.
- Safety auto-stop: phát hiện lost lane hoặc xe rời lane → tự set
  `car_enabled=False`.
- Publish `/car_enabled` (Bool) @ 10 Hz để MPC, EKF subscribe trạng thái.
- HW-independent — hiện chỉ dùng cho sim.

Tần suất:
- `/car_enabled` publish @ 10 Hz.
- `check_timeout` @ 2 Hz.
- Keyboard thread: blocking `readline`.

---

## 2. CẤU TRÚC FILE

- Hằng số: `LANE_STOP_TIMEOUT=1.5`, `LANE_OUT_TIMEOUT=1.5`, `N_MAX_STOP=0.22`.
- `MAP_CONFIGS`: dict map_id → spawn config.
- Helper: `yaw_to_quat(yaw)`.
- Class `ResetNode(Node)`:
  - `__init__` — declare, subs, pubs, service, keyboard thread.
  - `publish_enabled` / `lane_cb` / `check_timeout`.
  - `reset_srv_cb` — handler service.
  - `_keyboard_listener` — thread.
  - `do_reset` — teleport.
  - `_publish_stop` — helper zero velocity + steering.

---

## 3. GIẢI THÍCH KHỐI CODE

### 3.1 Constants & Map configs

```python
LANE_STOP_TIMEOUT = 1.5   # giây không detect lane → auto STOP
LANE_OUT_TIMEOUT  = 1.5   # giây |e_y| > N_MAX → auto STOP
N_MAX_STOP        = 0.22  # [m] — tăng 0.18→0.22: tránh false trigger DLC
```
- 0.22 m ~ 44% lane width (lane=0.5 m). Đủ rộng để xe sang làn DLC mà
  không auto-stop nhầm.

```python
MAP_CONFIGS = {
    1: {'world':'lane_track',     'x':0.0, 'y':1.0,    'z':0.05, 'yaw':0.0},
    2: {'world':'track_test',     'x':0.0, 'y':-2.666, 'z':0.05, 'yaw':0.0},
    3: {'world':'lane_change',    'x':0.0, 'y':0.75,   'z':0.05, 'yaw':0.0},
    4: {'world':'obstacle_track', 'x':1.5, 'y':1.0,    'z':0.05, 'yaw':0.0},
}
```
- Spawn point phải trùng lane ở yaw 0 (hướng +X) cho mỗi world SDF.

### 3.2 `__init__`

```python
self.declare_parameter('map_id', 1)
map_id = self.get_parameter('map_id').value
if map_id not in MAP_CONFIGS:
    raise SystemExit(1)
```
Fail-fast nếu map_id không hợp lệ.

```python
self.pub_vel     = self.create_publisher(Float64, '/velocity', 10)
self.pub_delta   = self.create_publisher(Float64, '/steering_angle', 10)
self.pub_enabled = self.create_publisher(Bool, '/car_enabled', 10)
```
- `/velocity`, `/steering_angle`: overlap publisher với MPC — khi
  `car_enabled=False`, reset_node publish 0 để override MPC nếu còn đang chạy.
  Lưu ý: ROS2 không có priority, frame nào đến sau cùng thắng. Muốn an toàn
  tuyệt đối → phải tắt MPC hoặc filter ở bridge.

```python
threading.Thread(target=self._keyboard_listener, daemon=True).start()
```
- Daemon thread để Ctrl+C main thread thoát được. Không dùng `rclpy.timer`
  vì `readline` blocking.

### 3.3 `lane_cb` & `check_timeout`

```python
def lane_cb(self, msg):
    if msg.lane_detected and self.car_enabled:
        self.last_detected_time = self.get_clock().now()
        self.resetting = False
        if abs(msg.e_y) < N_MAX_STOP:
            self.last_inlane_time = self.get_clock().now()
```
- Chỉ cập nhật timestamp khi `car_enabled=True` — tránh auto-stop khi xe
  đang chờ start.
- Tách 2 timestamp: `last_detected_time` (có lane) và `last_inlane_time`
  (lane + xe gần tâm).

```python
def check_timeout(self):
    if not self.car_enabled: return
    ...
    if elapsed_lost > LANE_STOP_TIMEOUT:
        self.car_enabled = False
        self._publish_stop()
    elif elapsed_out > LANE_OUT_TIMEOUT:
        self.car_enabled = False
        self._publish_stop()
```
Auto-stop 2 case: (a) lost lane >1.5 s, (b) rời lane >0.22 m >1.5 s. Log
warn để user biết nguyên nhân.

### 3.4 `_keyboard_listener`

```python
if not sys.stdin.isatty():
    self.get_logger().info('stdin khong phai TTY — dung service')
    return
while True:
    key = sys.stdin.readline().strip().lower()
    if key == 's': self.car_enabled = True; ...
    elif key == 'x': self.car_enabled = False; self._publish_stop()
    elif key == 'q': self.do_reset()
```
- Bypass khi stdin không phải TTY (chạy qua systemd/service) → thread return,
  chỉ còn interface service `/reset_car`.
- `readline().strip().lower()` nên phím `S<Enter>` hay `s<Enter>` đều được.

### 3.5 `do_reset` — teleport qua Ignition service

```python
for _ in range(5):
    self._publish_stop()
    time.sleep(0.1)
```
Publish stop 5 lần (500 ms) trước teleport để physics dừng hoàn toàn — nếu
teleport khi bánh xe đang quay, Gazebo có thể bay xe do momentum.

```python
req = (
    f'name: "ackermann_steering_vehicle" '
    f'position: {{x: {self.spawn_x} y: {self.spawn_y} z: {self.spawn_z}}} '
    f'orientation: {{x: {self.qx} y: {self.qy} z: {self.qz:.4f} w: {self.qw:.4f}}}'
)
```
- **Gotcha Fortress**: protobuf text format BẮT BUỘC có dấu `:` trước nested
  (`position: {...}`), KHÔNG phải `position {...}` như các version khác.
  Sai format → Fortress parse fail. Xem CLAUDE.md §5.
- Tên model `ackermann_steering_vehicle` khớp với model Gazebo spawn từ xacro.

```python
cmd_list = ['ign', 'service',
    '-s', f'/world/{self.world}/set_pose',
    '--reqtype', 'ignition.msgs.Pose',
    '--reptype', 'ignition.msgs.Boolean',
    '--timeout', '2000',
    '--req', req]
result = subprocess.run(cmd_list, capture_output=True, text=True,
                        env=os.environ, timeout=5.0)
```
- Dùng **list** thay `shell=True` → tránh quoting bug (tên có space, etc).
- `env=os.environ` để kế thừa `IGN_GAZEBO_RESOURCE_PATH` nếu có.
- `ign` (không phải `gz`) cho Fortress. CLI `gz` là của Garden+.

Check kết quả:
```python
if result.returncode != 0 or 'data: false' in stdout:
    self.get_logger().error(...)
```
Ignition reply `ignition.msgs.Boolean {data: true/false}` — `data: false`
nghĩa là service không tìm được entity.

### 3.6 `_publish_stop`

```python
self.pub_vel.publish(Float64(data=0.0))
self.pub_delta.publish(Float64(data=0.0))
```
Atomic zero publish. Được gọi nhiều chỗ: khi STOP, khi auto-stop timeout, khi
teleport, ở đầu `__init__`.

---

## 4. TOÁN HỌC

`yaw_to_quat(yaw)` dùng công thức quaternion quanh trục z:
`(x, y, z, w) = (0, 0, sin(yaw/2), cos(yaw/2))`.

---

## 5. THAM SỐ

- `map_id` (int, default 1): 1/2/3/4 ứng với 4 world SDF.
- Hardcoded: `LANE_STOP_TIMEOUT`, `LANE_OUT_TIMEOUT`, `N_MAX_STOP`.

---

## 6. GOTCHAS

- **Fortress text format** — xem §3.5, dễ sai nhất. Nếu CI/tool upgrade lên
  Gazebo Garden, phải đổi `ign` → `gz` và format msg `gz.msgs.*`.
- **Keyboard thread không thoát** khi main dead — dùng `daemon=True`. Lưu ý:
  khi detach qua tmux/nohup mà stdin vẫn open, thread có thể đọc rác.
- **Publish `/velocity`, `/steering_angle` đè MPC**: chỉ hoạt động vì reset
  node publish 0 sau MPC publish — race condition. Best practice: MPC nên
  check `car_enabled` trước publish (hiện đã làm).
- **`MAP_CONFIGS` hardcoded** — đổi world phải sửa tay. TODO externalize.
- **Auto-stop `N_MAX_STOP=0.22`** khá nhạy với DLC. Nếu tune DLC amplitude
  > 0.22 m, phải nới value (hoặc tắt check).
- **`rclpy.ok()` check** trong `finally` — tránh double shutdown error khi
  Ctrl+C đã shutdown.

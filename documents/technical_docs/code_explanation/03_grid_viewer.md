# 03 — grid_viewer.py

File: `src/perception_pkg/perception_pkg/grid_viewer.py` (42 dòng)

---

## 1. MỤC ĐÍCH

- Utility viewer chạy tay trên máy local — không phải node trong pipeline.
- Subscribe `/perception/debug_grid` (2×2 view từ `visualizer_node`) và
  hiển thị bằng `cv2.imshow()` cửa sổ native OpenCV.
- Dùng khi Foxglove/rqt không tiện, muốn xem nhanh trên máy có X server.
- Tần suất: theo callback ảnh (~30 Hz).

---

## 2. CẤU TRÚC FILE

- Class `GridViewer(Node)`:
  - `__init__`: CvBridge + sub + namedWindow.
  - `cb(msg)`: imshow + waitKey.
- `main()`: rclpy boilerplate + cleanup cửa sổ.

---

## 3. GIẢI THÍCH

### 3.1 `__init__`

```python
self.bridge = CvBridge()
self.sub = self.create_subscription(
    Image, "/perception/debug_grid", self.cb, 10)
cv2.namedWindow("Perception Debug [2x2]", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Perception Debug [2x2]", 1280, 1040)
```

- `CvBridge()` chuyển `sensor_msgs/Image` → `numpy.ndarray BGR`.
- QoS depth=10 đủ (không phải best-effort — chấp nhận trễ chút để không drop).
- `WINDOW_NORMAL` cho phép resize bằng chuột. Kích thước mặc định 1280×1040
  đủ chỗ cho 2×2 view native 642×522 + margin.

### 3.2 `cb(msg)`

```python
img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
cv2.imshow("Perception Debug [2x2]", img)
cv2.waitKey(1)
```

- `waitKey(1)` bắt buộc — OpenCV GUI cần ≥ 1 ms event-loop để render và xử
  lý keyboard. Thiếu `waitKey` → cửa sổ đóng băng.
- Không xử lý phím bấm — muốn quit thì Ctrl+C terminal.

### 3.3 `main()`

```python
rclpy.init()
node = GridViewer()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
cv2.destroyAllWindows()
node.destroy_node()
rclpy.shutdown()
```

- Bắt `KeyboardInterrupt` để dọn sạch trước khi exit — tránh zombie window.
- Thứ tự cleanup: cv2 window trước, rclpy sau.

---

## 4. THUẬT TOÁN / TOÁN HỌC

Không có — chỉ là lớp bridge ROS2 → GUI OpenCV.

---

## 5. THAM SỐ YAML

Không có param.

---

## 6. GOTCHAS

- **Cần X server** — chạy SSH phải `ssh -X` hoặc chạy tại máy có display.
  Trên Pi headless → không dùng được, thay bằng Foxglove.
- **Không đăng ký trong `setup.py` entry_points** — phải chạy tay
  `python3 grid_viewer.py` (xem docstring). Nếu muốn `ros2 run`, thêm vào
  `perception_pkg/setup.py`.
- **Không tích hợp vào launch file** — pure dev tool, không cần launch.
- QoS `depth=10` khác với publisher `depth=1` best-effort của
  `visualizer_node` → ROS2 sẽ vẫn nhận được (compatibility giữa RELIABLE sub
  và BEST_EFFORT pub: **sub không nhận** nếu sub khai RELIABLE). Default của
  `create_subscription` là RELIABLE → có thể miss frame. Nếu không thấy ảnh,
  đổi sub sang BEST_EFFORT match publisher.

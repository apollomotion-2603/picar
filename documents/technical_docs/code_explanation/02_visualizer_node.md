# 02 — visualizer_node.py

File: `src/perception_pkg/perception_pkg/visualizer_node.py` (286 dòng)

---

## 1. MỤC ĐÍCH

- Node debug thuần tuý — **không** feed vào control loop.
- Build 4 view trực quan cho pipeline perception rồi gộp thành grid 2×2 để
  xem bằng Foxglove / rqt_image_view.
- Input: `/camera/image_raw` + `/perception/lane_state` (overlay giá trị
  thật từ perception_node).
- Output: 5 topic Image — `view1_raw`, `view2_bev`, `view3_threshold`,
  `view4_lanes`, `debug_grid`.
- Tần suất: callback theo ảnh, QoS `BEST_EFFORT depth=1` để giảm lag.

---

## 2. CẤU TRÚC FILE

- Constants layout: `GRID_W=320`, `GRID_H=260`, 6 màu BGR.
- Helper: `put_text`, `to_cell`.
- Class `VisualizerNode(Node)`:
  - `__init__`: params + pub/sub, lane_state cache.
  - `callback(msg)`: build 4 view + grid.
  - `_sliding_window(binary)`: biến thể có trả thêm boxes.
  - `_lane_state_cb(msg)`: cache lane state.

---

## 3. GIẢI THÍCH KHỐI CODE

### 3.1 Helpers

```python
def put_text(img, text, row, color=(255,255,255), x0=5, y0=20, dy=18):
    cv2.putText(img, text, (x0, y0+row*dy), font, 0.42, (0,0,0), 3)
    cv2.putText(img, text, (x0, y0+row*dy), font, 0.42, color,   1)
```
Double-stroke: vẽ viền đen dày 3px rồi chữ màu 1px → đọc được trên nền
sáng/tối bất kỳ.

```python
def to_cell(img):
    return cv2.resize(img, (GRID_W, GRID_H))
```
Đưa mọi view về cùng `320×260` để ghép grid.

### 3.2 Params + QoS

- Declare cùng 10 param với perception_node (cần khớp BEV giữa 2 node).
- **Gotcha CLAUDE.md đã ghi:** `bev_dst` default hardcode `[50,...,250,...]`
  (lane 200 px trong BEV 300) — khác với yaml thực tế `[100,...,300,...]`
  trong BEV 400. Khi đổi `perception.yaml` phải chắc chắn launch file truyền
  param-file giống nhau cho cả hai node.

```python
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1)
```
BEST_EFFORT + depth=1 → bỏ frame cũ, không block. Lý do: visualizer không
cần reliable, ưu tiên latency.

### 3.3 `callback(msg)` — View 1 (Raw + ROI)

```python
roi_pts = self.SRC.astype(np.int32).reshape((-1,1,2))
overlay = v1.copy()
cv2.fillPoly(overlay, [roi_pts], (0,255,0))
cv2.addWeighted(overlay, 0.15, v1, 0.85, 0, v1)
cv2.polylines(v1, [roi_pts], True, CLR_ROI, 2)
```
Fill polygon trapezoid xanh 15% rồi alpha-blend lên ảnh gốc → người dùng
thấy vùng ROI nhưng vẫn đọc được nội dung. `polylines` vẽ viền, `circle`
vẽ 4 corner để calibrate bằng mắt.

### 3.4 View 2 — BEV + grid

Warp BEV rồi vẽ grid 50 px + đường giữa xanh (ref line) để so lane position.

### 3.5 View 3 — Threshold + Histogram

```python
hist = np.sum(binary[self.BEV_H//2:,:],axis=0).astype(np.float32)
hist_norm = (hist/(hist.max()+1e-6)*50).astype(int)
for x,h in enumerate(hist_norm):
    cv2.line(v3,(x,self.BEV_H),(x,self.BEV_H-h),(0,180,255),1)
```
Vẽ histogram cột dưới đáy ảnh, normalize để peak cao max 50 px. 2 đỉnh tương
ứng 2 lane → dễ debug khi seed window sai.

### 3.6 View 4 — Lanes + overlay giá trị thật

```python
left_pts, right_pts, boxes = self._sliding_window(binary)
```
Biến thể `_sliding_window` trả thêm `boxes` để vẽ khung.

Fit **approximate** riêng trong visualizer:
```python
pl = np.polyfit([p[1] for p in left_pts], [p[0] for p in left_pts], 3)
pr = np.polyfit([p[1] for p in right_pts], [p[0] for p in right_pts], 3)
pc = (pl+pr)/2.0
```
- Fit `x = f(y)` (pixel space), trung bình 2 polynomial để có centerline.
- **Khác với perception_node**: node chính fit theo arc-length `x(s)`, còn
  visualizer fit pixel → giá trị `e_y, e_psi, kappa` ở đây chỉ là gần đúng,
  in kèm prefix `~` để phân biệt.
- Công thức approximate: `e_y = (x_c(y_bot) − BEV_W/2) · scale`,
  `e_psi = arctan((dx/dy) · scale)`, `kappa ≈ 2·pc[1]·scale²` (chỉ đúng
  khi `dx/dy` nhỏ).

Overlay giá trị THẬT từ `lane_state`:
```python
ls = self.lane_state
if ls is not None and ls.lane_detected:
    for sv in np.linspace(0.0, min(ls.s_max, 0.65), 40):
        xc_m = ls.coeff_a*sv**3 + ls.coeff_b*sv**2 + ls.coeff_c*sv + ls.coeff_d
        xc_px = int(xc_m/self.SCALE + self.BEV_W/2)
        yc_px = int(self.BEV_H * (1.0 - sv / (ls.s_max if ls.s_max > 0.05 else 0.7)))
```
- Sample 40 điểm dọc `s ∈ [0, min(s_max, 0.65)]`.
- Convert `x(s)` mét → pixel (gốc tâm đáy BEV).
- Convert `s` → pixel `y` bằng scale `y = BEV_H · (1 − s/s_max)` — xấp xỉ
  tuyến tính vì arc-length ≠ forward-pixel (sai số nhỏ khi đường gần thẳng).
  **Line 192–193 dead code** (`yc_px = int(self.BEV_H − sv/self.SCALE *
  self.SCALE / self.SCALE * …)`) bị line 194 ghi đè — cleanup debt.
- Vẽ polyline xanh lá đậm `(0,255,0)` → phân biệt với approximate
  centerline màu vàng `CLR_CENTER`.

Text overlay (3 dòng khi lane_detected):
```
e_y : +12.3mm
epsi: +1.2d
kap : +0.0345
```

Mode text:
- Có ls + detected → 3 dòng xanh lá.
- Có ls + LOST → `LOST` đỏ.
- Chưa có ls → `wait lane_state` xám.
- Không có ls nhưng fit pixel được → 3 dòng `~e_y, ~epsi, ~kap` vàng.

### 3.7 Ghép grid 2×2

```python
top = np.concatenate([c1, np.zeros((GRID_H,2,3),np.uint8)+60, c2], axis=1)
bot = np.concatenate([c3, np.zeros((GRID_H,2,3),np.uint8)+60, c4], axis=1)
div = np.zeros((2, GRID_W*2+2, 3), np.uint8)+60
grid = np.concatenate([top, div, bot], axis=0)
```
Ghép 2 cell + 2 px divider xám 60 theo trục 1 cho mỗi hàng, rồi nối 2 hàng
+ divider theo trục 0. Kích thước cuối: `(260·2+2, 320·2+2, 3) = 522×642`.

Label góc: `RAW+ROI | BEV | THRESH | LANES`.

### 3.8 `_sliding_window` vs perception_node

- **Giống**: histogram seed, 10 window, update `left_x/right_x` theo mean.
- **Khác**:
  - Không trả `center_pts` (vì visualizer tự fit từ `pl/pr`).
  - Trả thêm `boxes = [(xl1,xl2,xr1,xr2,yt,yb), ...]` để vẽ rectangle.
- Cần đồng bộ tay nếu perception_node đổi thuật toán sliding window.

---

## 4. THAM SỐ YAML

Giống `perception_node` — dùng chung `perception.yaml`.

---

## 5. KNOWN ISSUES / GOTCHAS

- **Dead code line 192–193**: phép tính `yc_px` thừa, bị line 194 đè. Không
  gây bug nhưng khó đọc — có thể xoá.
- **Approximate metrics khác real metrics**: `~e_y` trong view 4 (fit pixel)
  có thể khác `ls.e_y` (fit arc-length) đặc biệt khi cua. Text màu khác nhau
  (vàng vs xanh lá) là dấu hiệu phân biệt.
- **Khi đổi `bev_scale`, `bev_width`**: remember rằng default hardcode ở
  `__init__` khác yaml — luôn launch với `--ros-args --params-file`, đừng
  dựa vào default.
- **Sync với perception_node**: thuật toán `_sliding_window` nhân bản từ
  perception_node nhưng không chia sẻ — fix bug ở node chính phải fix cả đây.
- **QoS ảnh input depth=1 best-effort**: phù hợp debug, **không** thích hợp
  cho bag replay (có thể drop frame → mismatch với lane_state).
- File header ghi `bev_dst: [50,0,250,0,...]` default nhưng yaml thật dùng
  `[100,0,300,0,...]` — hiện tượng default lỗi thời.

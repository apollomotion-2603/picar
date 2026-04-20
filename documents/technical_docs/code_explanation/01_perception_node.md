# 01 — perception_node.py

File: `src/perception_pkg/perception_pkg/perception_node.py` (236 dòng)

---

## 1. MỤC ĐÍCH

- Nhận ảnh RGB thô từ camera (`/camera/image_raw`), trích xuất centerline của
  làn đường ở toạ độ xe (vehicle frame), publish ra `/perception/lane_state`.
- Input: `sensor_msgs/Image` từ Pi Camera 3 (sim: Gazebo camera plugin).
- Output: `lane_msgs/LaneState` gồm `e_y, e_psi, kappa, coeff_a..d, s_max,
  lane_detected` — feed thẳng vào MPC.
- Tần suất: chạy theo callback ảnh (~30 Hz sim, ~20–30 Hz HW tuỳ driver CSI).

Pipeline: `RGB → BEV warp → Gray → Blur → Adaptive Threshold → Sliding
Window → Cubic fit arc-length → LaneState`.

---

## 2. CẤU TRÚC FILE

- Hằng số: `LANE_WIDTH_M = 0.5`.
- Hàm tự do: `fit_cubic_arclength(pts_px, bev_w, bev_h, scale, s_max_m, n_arc)`.
- Class `PerceptionNode(Node)`:
  - `__init__`: declare params, build homography, tạo sub/pub.
  - `callback(msg)`: BEV → threshold → sliding_window → fit → publish.
  - `sliding_window(binary)`: tách 2 lane trái/phải + center theo window.
- `main()`: rclpy boilerplate.

---

## 3. GIẢI THÍCH TỪNG KHỐI

### 3.1 Hằng số lane width

```python
LANE_WIDTH_M = 0.5   # m — khoảng cách 2 làn thật
```

Bề rộng lane vật lý. Dùng cho fallback khi chỉ thấy 1 lane (offset centerline
±0.25 m). Nếu thay track → chỉnh ở đây + `bev_dst` trong yaml.

### 3.2 `fit_cubic_arclength` — fit centerline theo arc-length

```python
xs_m = np.array([(p[0] - bev_w / 2) * scale for p in pts_px])
us_m = np.array([(bev_h - p[1]) * scale for p in pts_px])
order = np.argsort(us_m)
us_m = us_m[order]
xs_m = xs_m[order]
if len(us_m) < 4:
    return None
```

- Chuyển pixel BEV → mét, đặt gốc ở tâm dưới ảnh BEV (xe ở `(0, 0)`).
- `xs_m` = lateral offset (trái−, phải+). `us_m` = forward distance (pixel Y
  ngược nên lấy `bev_h − p[1]`).
- Cần ≥ 4 điểm để fit cubic.

```python
within = us_m <= s_max_m
us_fit = us_m[within] if np.sum(within) >= 4 else us_m
xs_fit = xs_m[within] if np.sum(within) >= 4 else xs_m
```

FIX: chỉ fit điểm trong `s_max_m` (≈0.7 m). Điểm xa hơn bị sliding window
drift → gây bowing (cong giả) khi cua.

```python
half = max(float(us_fit[-1]) * 0.5, 0.05)
w_u = np.exp(-us_fit / half)
coeff_u = np.polyfit(us_fit, xs_fit, 3, w=w_u)
```

Near-field exponential weighting: điểm gần xe tin cậy hơn. Trọng số giảm
theo `exp(−u / half)`, `half` = nửa khoảng forward. Fit cubic `x = f(u)`
trong pixel-forward frame.

```python
u_dense = np.linspace(0.0, min(float(us_fit[-1]), s_max_m), n_arc)
dxdu = np.polyval(np.polyder(coeff_u), u_dense)
ds_du = np.sqrt(1.0 + dxdu ** 2)
s_dense = np.concatenate(([0.0],
    np.cumsum(np.diff(u_dense) * (ds_du[:-1] + ds_du[1:]) / 2.0)))
xc_dense = np.polyval(coeff_u, u_dense)
```

- Resample 200 điểm đều theo `u`.
- Tính arc-length tích phân: `s = ∫ √(1 + (dx/du)²) du`, dùng trapezoidal
  rule (`(ds_du[i] + ds_du[i+1]) / 2 · Δu`).
- Ý nghĩa: chuyển tham số từ `u` (forward) sang `s` (arc-length thực) — MPC
  dùng `s` là path parameter theo paper Kloeser 2020.

```python
half_s = max(s_max * 0.5, 0.05)
w_s = np.exp(-s_dense / half_s)
coeff_s = np.polyfit(s_dense, xc_dense, 3, w=w_s)

coeff_q = np.polyfit(s_dense, xc_dense, 2, w=w_s)
b_q = float(coeff_q[0])
c_q = float(coeff_q[1])
kappa = float((2.0 * b_q) / (1.0 + c_q ** 2) ** 1.5)
```

- `coeff_s` (cubic bậc 3) → publish `a, b, c, d` cho MPC tính horizon kappa.
- `coeff_q` (quadratic bậc 2) → tính `kappa₀` (curvature tại xe). Lý do tách:
  cubic fit tốt centerline nhưng đạo hàm 2 của cubic dao động mạnh → kappa
  nhiễu. Quadratic ít overfit nên kappa mượt hơn.
- Công thức kappa tại `s=0`: `κ = 2b / (1 + c²)^(3/2)` (xem mục 4).

### 3.3 `__init__` — params + homography

Declare 11 param; default là giá trị SIM (BEV corners đã calibrate cho
Gazebo camera). Tính `LANE_W_PX = 200` cho bev_scale=0.0025. Build
`cv2.getPerspectiveTransform(SRC, DST)` → ma trận H 3×3, cache lại cho mỗi
frame (không recompute).

Subscribe `/camera/image_raw`, publish `/perception/lane_state`, QoS depth 10.

### 3.4 `callback(msg)` — pipeline ảnh

1. `cv2.warpPerspective(img, H, (BEV_W, BEV_H))` — warp sang BEV.
2. `cvtColor → BGR2GRAY → GaussianBlur(5,5)`.
3. `adaptiveThreshold(GAUSSIAN_C, INV, block=35, C=10)` — làn trắng trên
   nền đen (invert). Adaptive để robust với shadow.
4. `sliding_window(binary)` → `(left_pts, right_pts, center_pts)`.

Mode selection:
- `n_center ≥ 4` → **NORMAL**, fit `center_pts`, offset=0.
- `n_left ≥ 4` → **LEFT_ONLY**, fit `left_pts`, offset=+0.25 m.
- `n_right ≥ 4` → **RIGHT_ONLY**, fit `right_pts`, offset=−0.25 m.
- else → **LOST**, `lane_detected=False`.

Sau fit: `e_y = d + offset`, `e_psi = arctan(c)`. Lưu ý `e_y` = lateral
error tại `s=0`, `e_psi` = heading error (góc tiếp tuyến centerline).

### 3.5 `sliding_window(binary)`

```python
hist    = np.sum(binary[self.BEV_H // 2:, :], axis=0)
mid     = self.BEV_W // 2
left_x  = int(np.argmax(hist[:mid]))
right_x = int(np.argmax(hist[mid:])) + mid
```

Histogram cột trên nửa dưới BEV → tìm đỉnh trái/phải làm seed cho window
đầu tiên.

Với mỗi window (10 windows, dưới lên):
- `roi_l = binary[y_top:y_bot, xl1:xl2]`, tương tự `roi_r`.
- Nếu `count > MIN_PIX` → update `left_x/right_x` theo mean, append điểm.
- **Chỉ append `center_pts` khi CẢ HAI lane detect cùng window**. Fix bug
  cũ: `zip(left, right)` ghép sai Y khi window miss không đều giữa 2 bên.

---

## 4. TOÁN HỌC

- BEV pixel → vehicle frame (m):
  `x = (pₓ − BEV_W/2) · scale`, `u = (BEV_H − p_y) · scale`.
- Arc-length: `s(u) = ∫₀ᵘ √(1 + f'(τ)²) dτ`, rời rạc hoá trapezoidal.
- Curvature từ `x = a + b·s + c·s² + d·s³`:
  `κ(s) = x''(s) / (1 + x'(s)²)^(3/2)`.
  Tại `s=0` (quadratic `x = c₀ + c₁·s + c₂·s²`): `κ₀ = 2·c₂ / (1 + c₁²)^(3/2)`.
  Trong code: `coeff_q = [b_q, c_q, const]` (numpy highest-first), nên
  `b_q` thực chất là hệ số `s²` → đó là lý do `kappa = 2·b_q / (1+c_q²)^1.5`.
- `e_psi = arctan(dx/ds|_{s=0}) = arctan(c)` (c là hệ số `s¹` của cubic
  `coeff_s = [a, b, c, d]` với `d = x(0)`).

---

## 5. THAM SỐ YAML

| Param | Default sim | Ý nghĩa |
|---|---|---|
| `bev_src` | 4 pixel góc (172,151)… | Trapezoid trong ảnh gốc |
| `bev_dst` | 4 pixel hình chữ nhật | Rectangle sau warp |
| `bev_width` | 400 | BEV px (map_1: 400) |
| `bev_height` | 500 | BEV px |
| `bev_scale` | 0.0025 | m/px — 0.5 m = 200 px |
| `n_windows` | 10 | Số sliding window |
| `win_width` | 100 (DLC) / 80 (oval) | Bề rộng window px |
| `min_pixels` | 20 (DLC) / 15 (oval) | Ngưỡng nhận lane |
| `s_max_m` | 0.7 | Fit horizon (m) |
| `n_arc` | 200 | Điểm resample arc-length |
| `thresh_block_size` | 35 | Adaptive threshold block |
| `thresh_c` | 10 | Adaptive threshold offset |

---

## 6. GOTCHAS

- `coeff_d` là `x(0)` (intercept), `coeff_a` là hệ số bậc 3 — numpy
  convention. MPC phải đọc đúng thứ tự `[a,b,c,d]` tương ứng `a·s³+b·s²+c·s+d`.
- Offset sign: LEFT_ONLY → centerline nằm BÊN PHẢI left lane → `e_y` (lateral
  của xe so với centerline) cần **cộng** `+LANE_WIDTH/2`. Dễ ngược dấu khi
  refactor.
- BEV_W trong code default 300 nhưng yaml override 400 → **luôn load yaml**,
  đừng tin default.
- `visualizer_node.py:17-18` có hardcoded BEV corners → khi đổi
  `perception.yaml` phải sync tay (TODO ghi trong CLAUDE.md).
- `fit_cubic_arclength` return `None` khi < 4 điểm hoặc `s_max < 0.05 m` →
  callback phải check.

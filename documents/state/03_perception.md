# 03. Perception Pipeline

Updated 2026-04-19.

**Source:** `src/perception_pkg/perception_pkg/perception_node.py`.
**Config:** `src/perception_pkg/config/perception.yaml`.

---

## Processing chain

```
/camera/image_raw (640×480, BGR)
  │
  ├─ BEV warp: cv2.warpPerspective(img, H, (400, 500))
  │    H = cv2.getPerspectiveTransform(SRC, DST)
  │
  ├─ cv2.cvtColor(BGR → GRAY)
  ├─ cv2.GaussianBlur(5, 5)
  ├─ cv2.adaptiveThreshold(GAUSSIAN_C, BINARY_INV,
  │                         blockSize=35, C=10)
  │
  ├─ Sliding Window (N_WIN=10, WIN_W=100px, MIN_PIX=20)
  │    ├─ histogram peak → left_x0, right_x0
  │    └─ slide bottom→top, recenter mỗi window ≥ MIN_PIX
  │
  ├─ Center points = midpoint(left_pts, right_pts)
  │    NOTE: chỉ lấy từ windows có CẢ HAI lane detect CÙNG Y
  │          (fix bug zip-by-index cũ)
  │
  ├─ Arc-length cubic fit (fit_cubic_arclength):
  │    pixel → metric (scale 0.0025 m/px)
  │    polyfit(u_dense, x_center, deg=3) in u-space
  │    reparametrize by arc-length s
  │    polyfit(s_dense, x_center, deg=3) → [a, b, c, d]
  │    clip us_m ≤ s_max_m; near-field exp weighting
  │
  ├─ κ từ quadratic fit (bậc 2), KHÔNG phải cubic
  │    → ít overfit, không spike
  │
  └─ Publish /perception/lane_state:
       e_y   = coeff_d
       e_psi = arctan(coeff_c)
       kappa = 2b_quad / (1 + c²)^1.5   (smooth hơn)
       coeff_a, b, c, d  (từ cubic fit, cho MPC tham chiếu nếu cần)
       s_max = s_max_m
```

---

## BEV homography (sim — recalibrate cho hardware)

```yaml
# perception.yaml
bev_src: [172.0, 151.0, 471.0, 151.0, 609.0, 365.0, 29.0, 368.0]
# SRC corners (pixel): TL, TR, BR, BL trong ảnh camera

bev_dst: [100.0, 0.0, 300.0, 0.0, 300.0, 500.0, 100.0, 500.0]
# DST corners (pixel): lane 200px căn giữa BEV 400px (100→300)

bev_width: 400    # Lane 200px chiếm 50% → ít noise hơn
bev_height: 500
bev_scale: 0.0025 # m/px — lane 0.5m = 200px
```

---

## Sliding window params

```yaml
n_windows: 10     # số window
win_width: 100    # width [px] (tăng 80→100 tracking khoẻ hơn khi DLC)
min_pixels: 20    # minimum pixels để recenter (giảm false detect)
```

---

## Arc-length params

```yaml
s_max_m: 0.7          # camera lookahead [m] (2.3× thân xe)
n_arc: 200            # số điểm integration arc-length
thresh_block_size: 35
thresh_c: 10
```

---

## Validated results (sim, xe đứng yên trên đường thẳng)

- `e_y ≈ -0.001 m`
- `e_psi ≈ -0.018 rad`
- `kappa ≈ 0.059 1/m` (residual từ quadratic fit)
- Kappa sign-change qua S-curve đúng dấu ✅.

---

## Khác biệt giữa `perception_node` và `visualizer_node`

| Aspect | `perception_node` | `visualizer_node` |
|--------|-------------------|-------------------|
| BEV SRC/DST | đọc từ `perception.yaml` | **hardcoded** lines 17-18 |
| κ formula | `2b_quad / (1+c²)^1.5` (arc-length) | `2·pc[1]·SCALE²` (pixel-space approx) |
| Purpose | publish lane_state | debug grid display |

⚠️ Khi đổi `perception.yaml` BEV params PHẢI sync tay vào
`visualizer_node.py:17-18`.

---

## Polynomial bowing fix (đã apply)

Vấn đề cũ: sliding window drift ở upper windows (far-field) → cubic polyfit
bị kéo lệch ra ngoài → hình chữ V trong LANES panel → kappa spike.

Fix:
1. Clip `us_m ≤ s_max_m` trước polyfit — loại points ngoài lookahead.
2. Near-field exponential weighting `exp(-s/half)` cho cả 2 lần polyfit.
3. **κ từ quadratic fit (bậc 2)** thay vì cubic — ít overfit, không spike.
4. Cubic coeffs `a,b,c,d` vẫn publish (cho tham chiếu/κ_horizon nếu cần).

---

## Issues còn mở (chưa fix)

| # | Issue | Mức | Ảnh hưởng |
|---|-------|-----|-----------|
| 5 | Double exponential weighting (u-space + s-space) | MAJOR | Near-field bias quá mạnh, far-field gần bỏ |
| 6 | Sliding window drift ở upper windows | MODERATE | Tích lũy error window-over-window |
| 7 | κ noisy → EKF predict α̇ noisy → feedback loop | MODERATE | Amplify noise qua closed loop |

### Alternative κ sources (nếu cần tune thêm)

| Method | Idea | Pros | Cons |
|--------|------|------|------|
| Circle fit | Fit đường tròn qua center points → κ=1/R | Trực tiếp, robust | Chỉ 1 κ, không model S-curve |
| Menger | κ từ 3 điểm liên tiếp | Cực đơn giản | Nhạy noise |
| B-spline | Thay polynomial | Local control, ổn định | Phức tạp |
| RANSAC + polyfit | Loại outlier trước fit | Robust drift | Thêm latency |
| KF (e_y, e_psi, κ) | KF tracking thay EMA | Adapt gain | Cần tune Q, R |

Khuyến nghị: circle fit hoặc KF nếu hardware vẫn dao động.

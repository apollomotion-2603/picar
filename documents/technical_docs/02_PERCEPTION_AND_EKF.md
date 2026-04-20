# 02. Nhận diện Làn đường (Perception) & Ước lượng Trạng thái (EKF)

Tài liệu chi tiết cách hệ thống "nhìn" đường và "cảm nhận" trạng thái nội
tại qua Perception Pipeline và Extended Kalman Filter.

---

## 1. Perception Pipeline

### 1.1 Quy trình xử lý ảnh

1. **Homography (BEV warp)** — chuyển ảnh camera (640×480) thành
   Bird's Eye View (400×500 pixel, scale 0.0025 m/px) để loại bỏ phối
   cảnh. 4 điểm SRC/DST lấy từ file `perception.yaml`.

2. **Adaptive Thresholding** — `cv2.adaptiveThreshold` với
   Gaussian kernel 35×35, C=10 → robust trước biến đổi ánh sáng,
   tách vạch trắng/đen khỏi nền.

3. **Sliding Window** — 10 cửa sổ (width 100 px, min 20 pixels), trượt từ
   đáy ảnh lên, tìm và theo dõi pixel thuộc vạch trái / phải.
   Histogram peak của nửa dưới khởi tạo `left_x0, right_x0`.

4. **Center points** — chỉ lấy midpoint từ các window có **cả hai** vạch
   detect cùng Y (tránh lệch khi window miss một bên). Đây là fix quan
   trọng giảm bias trong DLC.

5. **Cubic Spline Fitting (arc-length parameterised)**
   - Fit cubic $x_c(u)$ theo pixel, lấy mẫu dense.
   - Integrate arc-length $s(u) = \int_0^u \sqrt{1+x_c'^2}\,du$.
   - Re-fit cubic $x_c(s) = as^3 + bs^2 + cs + d$.
   - Near-field exponential weighting `exp(-s/half)` ưu tiên pixel gần xe.

6. **Curvature từ quadratic fit** — fit $x_c(s)$ bậc 2 riêng để tính
   $\kappa = 2b_{quad}/(1+c_{quad}^2)^{3/2}$. Chủ ý **không** lấy từ cubic
   vì cubic overfit khi ít điểm, gây spike.

---

### 1.2 Thông điệp `LaneState`

```
e_y       = coeff_d                   # lateral offset tại s=0  [m]
e_psi     = arctan(coeff_c)           # heading error tại s=0   [rad]
kappa     = 2·b_quad / (1+c_quad²)^1.5  # curvature tại s=0  [1/m]
coeff_a/b/c/d                          # cubic polynomial
s_max                                  # camera lookahead [m]
lane_detected                          # bool
```

Publish @ 30 Hz.

---

### 1.3 Kết quả validate (sim, xe đứng yên đường thẳng)

- `e_y ≈ -0.001 m` ✅
- `e_psi ≈ -0.018 rad` ✅
- `kappa ≈ 0.059 1/m` (residual bias từ fit)
- Sign-change của `kappa` qua S-curve đúng dấu ✅.

---

## 2. Extended Kalman Filter (EKF)

### 2.1 Vai trò

- Làm mượt dữ liệu perception (camera nhiễu, sliding window drift).
- Fusion nhiều nguồn: camera (n, α), IMU (α̇), joint_states (v từ wheel speed).
- Cung cấp trạng thái đầy đủ cho NMPC: `n, α, v` (filtered) + `n_dot,
  alpha_dot` (predicted).

### 2.2 State vector

$$
\mathbf{x} = [n,\ \dot n,\ \alpha,\ \dot\alpha,\ v,\ \psi]
$$

Publish qua `VehicleState` msg @ 100 Hz (theo IMU rate).

### 2.3 Predict / Update

- **Predict** (mỗi bước IMU): propagate trạng thái theo mô hình kinematic
  xe đơn giản. Dùng `α̇` từ IMU gyro trực tiếp.
- **Update camera** (mỗi frame lane_state): đo `n, α` → Kalman update.
- **Update wheel encoder** (joint_states): đo `v` từ bánh sau trung bình.

### 2.4 Các bảo vệ (safety)

- **`ekf_healthy` flag** — `false` khi chưa có đủ sensor, hoặc uncertainty
  vượt ngưỡng. NMPC tự fallback sang raw perception / first-order v model.
- **Reset on enable** — khi `car_enabled` chuyển False → True, reset state
  về 0 (trừ `v` lấy từ encoder) để không tích luỹ bias lúc xe đứng yên.
- **κ EMA riêng** trong EKF predict (α̇ từ gyro cần κ để chuyển sang
  heading frame tham chiếu) — tách khỏi κ của NMPC để không feedback loop.

---

## 3. Tầm nhìn và Sự ổn định

### 3.1 Look-ahead

Cubic polynomial cho phép NMPC nội suy `κ(s)` tại mọi $s \in [0, s_{max}]$
trong horizon 1 giây. NMPC dùng `s_j = v \cdot j \cdot dt` để tính điểm
nhìn trước cho từng stage.

**Lưu ý:** công thức này phụ thuộc `v`, nên khi `v` thay đổi, phần khó
của filtering là khớp κ giữa các frames — đây là lý do per-stage EMA fail
và phải rollback sang scalar EMA + broadcast constant (xem mục
[Issues mở][issues]).

[issues]: ../state/05_status_issues.md

### 3.2 BEV symmetry

BEV đúng calib → lane markings song song trong ảnh debug → cubic fit bớt
bias → `n, κ` chính xác hơn → NMPC ít rung lắc. Khi deploy hardware, BƯỚC
recalibrate BEV là quan trọng nhất.

### 3.3 Polynomial bowing fix

Vấn đề cũ: sliding window drift ở upper windows → cubic polyfit bị kéo
lệch ra ngoài → hình chữ "V" trong panel LANES → κ spike.

Fix:
1. Clip `us_m ≤ s_max_m` trước polyfit.
2. Near-field exp weighting.
3. κ từ **quadratic fit** — ít overfit, không spike.
4. `kappa_max` clamp trong NMPC để chặn outlier còn sót.

---

## 4. Khác biệt `perception_node` vs `visualizer_node`

| Aspect | `perception_node` | `visualizer_node` |
|--------|-------------------|-------------------|
| Mục đích | publish state cho NMPC | hiển thị debug 4-panel |
| BEV SRC/DST | đọc từ `perception.yaml` | **hardcoded** lines 17-18 |
| κ formula | `2b_quad/(1+c²)^1.5` | `2·pc[1]·SCALE²` (approx) |

⚠️ **Khi đổi `perception.yaml` BEV phải sync tay vào visualizer**.

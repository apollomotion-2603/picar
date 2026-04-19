# 03. Bộ điều khiển NMPC 6 Trạng thái

Tài liệu này giải thích thuật toán Nonlinear Model Predictive Control (NMPC) dựa trên mô hình Bicycle 2D (Kloeser 2020).

## 1. Mô hình toán học (Spatial Bicycle Model)

Hệ thống sử dụng mô hình 6 trạng thái ($x$) và 2 đầu vào điều khiển ($u$):

- **States (x):** $[s, n, \alpha, v, D, \delta]$
  - $s$: Arclength (vị trí dọc).
  - $n$: Lateral error (vị trí ngang).
  - $\alpha$: Heading error (góc hướng).
  - $v$: Velocity (vận tốc).
  - $D$: Duty cycle (throttle).
  - $\delta$: Steering angle (góc lái).

- **Controls (u):** $[derD, derDelta]$ (Đạo hàm bậc nhất của $D$ và $\delta$).

Việc bao gồm $D$ và $\delta$ vào State vector giúp NMPC quản lý được tốc độ thay đổi của actuators (Rate Control), giúp xe chuyển động mượt mà hơn.

## 2. Giải thuật Solver (Acados)

NMPC được xây dựng trên framework **Acados**:
- **Solver Type:** SQP_RTI (Real-Time Iteration).
- **Hessian Approx:** Gauss-Newton.
- **QP Solver:** HPIPM (giải quyết bài toán tối ưu lồi ở mỗi bước lặp).
- **Integrator:** ERK (Explicit Runge-Kutta bậc 4).

## 3. Nội suy Curvature ($\kappa(s)$)

Đây là điểm cải tiến quan trọng giúp xe "biết trước" cua:
- NMPC lấy đa thức bậc 3 từ Perception.
- Tại mỗi giai đoạn (stage) $j$ trong Prediction Horizon ($N=20$), NMPC tính toán $\kappa(s_j)$ tương ứng.
- Điều này cho phép solver tối ưu hóa góc lái dựa trên hình dạng thực tế của đường phía trước.

## 4. Hàm Mục tiêu (Cost Function)

Hàm cost được thiết kế theo dạng Bình phương tối thiểu (Linear Least Squares):
- **Phạt $n$:** Rất nặng để xe luôn bám tâm đường.
- **Phạt $\alpha$:** Để xe luôn hướng song song với đường.
- **Phạt $u$:** Để hạn chế bẻ lái gắt hoặc tăng tốc đột ngột.
- **Reference $v$:** Tốc độ mục tiêu được điều chỉnh dựa trên độ cong của đường (Adaptive Speed).

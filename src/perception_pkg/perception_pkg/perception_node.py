import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_msgs.msg import LaneState
import cv2
import numpy as np

LANE_WIDTH_M = 0.5   # m — khoảng cách 2 làn thật

def fit_cubic_arclength(pts_px, bev_w, bev_h, scale, s_max_m, n_arc):
    xs_m = np.array([(p[0] - bev_w / 2) * scale for p in pts_px])
    us_m = np.array([(bev_h - p[1]) * scale for p in pts_px])
    order = np.argsort(us_m)
    us_m = us_m[order]
    xs_m = xs_m[order]
    if len(us_m) < 4:
        return None

    # FIX: Chỉ dùng điểm trong s_max_m — điểm xa bị sliding window drift gây bowing
    within = us_m <= s_max_m
    us_fit = us_m[within] if np.sum(within) >= 4 else us_m
    xs_fit = xs_m[within] if np.sum(within) >= 4 else xs_m

    # Near-field exponential weighting: điểm gần xe tin cậy hơn điểm xa
    half = max(float(us_fit[-1]) * 0.5, 0.05)
    w_u = np.exp(-us_fit / half)
    coeff_u = np.polyfit(us_fit, xs_fit, 3, w=w_u)

    u_dense = np.linspace(0.0, min(float(us_fit[-1]), s_max_m), n_arc)
    dxdu = np.polyval(np.polyder(coeff_u), u_dense)
    ds_du = np.sqrt(1.0 + dxdu ** 2)
    s_dense = np.concatenate(([0.0],
        np.cumsum(np.diff(u_dense) * (ds_du[:-1] + ds_du[1:]) / 2.0)))
    xc_dense = np.polyval(coeff_u, u_dense)
    s_max = float(s_dense[-1])
    if s_max < 0.05:
        return None

    # Cubic fit (weighted) cho e_y, e_psi, coeff tracking
    half_s = max(s_max * 0.5, 0.05)
    w_s = np.exp(-s_dense / half_s)
    coeff_s = np.polyfit(s_dense, xc_dense, 3, w=w_s)

    # Quadratic fit cho kappa: bậc 2 ít overfit hơn cubic → kappa mượt hơn
    coeff_q = np.polyfit(s_dense, xc_dense, 2, w=w_s)
    b_q = float(coeff_q[0])
    c_q = float(coeff_q[1])
    kappa = float((2.0 * b_q) / (1.0 + c_q ** 2) ** 1.5)

    return float(coeff_s[0]), float(coeff_s[1]), float(coeff_s[2]), float(coeff_s[3]), s_max, kappa


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('bev_src', [172.0,151.0,471.0,151.0,609.0,365.0,29.0,368.0])
        self.declare_parameter('bev_dst', [50.0,0.0,250.0,0.0,250.0,500.0,50.0,500.0])
        self.declare_parameter('bev_width', 300)
        self.declare_parameter('bev_height', 500)
        self.declare_parameter('bev_scale', 0.0025)
        self.declare_parameter('n_windows', 10)
        self.declare_parameter('win_width', 60)
        self.declare_parameter('min_pixels', 20)
        self.declare_parameter('s_max_m', 0.8)
        self.declare_parameter('n_arc', 200)
        self.declare_parameter('thresh_block_size', 35)
        self.declare_parameter('thresh_c', 10)

        src_flat = self.get_parameter('bev_src').value
        dst_flat = self.get_parameter('bev_dst').value
        self.BEV_W       = self.get_parameter('bev_width').value
        self.BEV_H       = self.get_parameter('bev_height').value
        self.SCALE       = self.get_parameter('bev_scale').value
        self.N_WIN       = self.get_parameter('n_windows').value
        self.WIN_W       = self.get_parameter('win_width').value
        self.MIN_PIX     = self.get_parameter('min_pixels').value
        self.S_MAX       = self.get_parameter('s_max_m').value
        self.N_ARC       = self.get_parameter('n_arc').value
        self.THRESH_BLOCK = self.get_parameter('thresh_block_size').value
        self.THRESH_C    = self.get_parameter('thresh_c').value

        # Lane width in pixels
        self.LANE_W_PX = int(LANE_WIDTH_M / self.SCALE)  # 0.5/0.0025 = 200px

        SRC = np.float32(src_flat).reshape(4, 2)
        DST = np.float32(dst_flat).reshape(4, 2)

        self.bridge = CvBridge()
        self.H = cv2.getPerspectiveTransform(SRC, DST)

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(
            LaneState, '/perception/lane_state', 10)

        self.get_logger().info(
            f'Perception node started (cubic + arc-length + single-lane fallback) | '
            f'BEV={self.BEV_W}x{self.BEV_H} scale={self.SCALE}m/px ' 
            f'lane_w={self.LANE_W_PX}px')

    def callback(self, msg):
        img    = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        bev    = cv2.warpPerspective(img, self.H, (self.BEV_W, self.BEV_H))
        gray   = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        blur   = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blur, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.THRESH_BLOCK, self.THRESH_C)

        left_pts, right_pts, center_pts = self.sliding_window(binary)

        lane_msg = LaneState()
        lane_msg.header.stamp    = self.get_clock().now().to_msg()
        lane_msg.header.frame_id = 'camera'

        n_left   = len(left_pts)
        n_right  = len(right_pts)
        n_center = len(center_pts)

        # ── Mode selection ──
        # NORMAL: dùng center_pts từ sliding_window (đã ghép đúng Y level)
        if n_center >= 4:
            mode = 'NORMAL'
            fit_pts = center_pts
            e_y_offset = 0.0

        elif n_left >= 4:
            mode = 'LEFT_ONLY'
            fit_pts = left_pts
            # Centerline = left lane + lane_width/2 sang phải (tăng x)
            e_y_offset = LANE_WIDTH_M / 2.0

        elif n_right >= 4:
            mode = 'RIGHT_ONLY'
            fit_pts = right_pts
            # Centerline = right lane - lane_width/2 sang trái (giảm x)
            e_y_offset = -LANE_WIDTH_M / 2.0

        else:
            mode = 'LOST'
            fit_pts = []
            e_y_offset = 0.0

        # ── Fit và publish ──
        if mode != 'LOST':
            result = fit_cubic_arclength(
                fit_pts, self.BEV_W, self.BEV_H,
                self.SCALE, self.S_MAX, self.N_ARC)

            if result is not None:
                a, b, c, d, s_max, kappa = result
                e_y   = float(d) + e_y_offset
                e_psi = float(np.arctan(c))

                lane_msg.e_y           = e_y
                lane_msg.e_psi         = e_psi
                lane_msg.kappa         = kappa   # từ quadratic fit (mượt hơn cubic)
                lane_msg.coeff_a       = a
                lane_msg.coeff_b       = b
                lane_msg.coeff_c       = c
                lane_msg.coeff_d       = d
                lane_msg.s_max         = s_max
                lane_msg.lane_detected = True

                if mode != 'NORMAL':
                    self.get_logger().warn(
                        f'Mode: {mode} | e_y={e_y*1000:.1f}mm (offset={e_y_offset*1000:.0f}mm)')
            else:
                lane_msg.lane_detected = False
                self.get_logger().warn(f'Arc-length fit failed [{mode}]')
        else:
            lane_msg.lane_detected = False
            self.get_logger().warn('Mode: LOST — no lane detected')

        self.pub.publish(lane_msg)

    def sliding_window(self, binary):
        """
        Sliding window lane detection.
        Returns: (left_pts, right_pts, center_pts)
        - center_pts chỉ chứa điểm từ windows có CẢ HAI lane detect (cùng Y level)
        - Fix bug zip ghép sai Y khi window miss không đều giữa 2 bên
        """
        hist    = np.sum(binary[self.BEV_H // 2:, :], axis=0)
        mid     = self.BEV_W // 2
        left_x  = int(np.argmax(hist[:mid]))
        right_x = int(np.argmax(hist[mid:])) + mid
        win_h   = self.BEV_H // self.N_WIN
        left_pts, right_pts, center_pts = [], [], []

        for w in range(self.N_WIN):
            y_top = self.BEV_H - (w + 1) * win_h
            y_bot = self.BEV_H - w * win_h
            y_mid = (y_top + y_bot) // 2

            xl1 = max(left_x  - self.WIN_W // 2, 0)
            xl2 = min(left_x  + self.WIN_W // 2, self.BEV_W)
            xr1 = max(right_x - self.WIN_W // 2, 0)
            xr2 = min(right_x + self.WIN_W // 2, self.BEV_W)
            roi_l = binary[y_top:y_bot, xl1:xl2]
            roi_r = binary[y_top:y_bot, xr1:xr2]
            px_l  = np.where(roi_l > 0)
            px_r  = np.where(roi_r > 0)

            left_found = len(px_l[1]) > self.MIN_PIX
            right_found = len(px_r[1]) > self.MIN_PIX

            if left_found:
                left_x = int(np.mean(px_l[1])) + xl1
                left_pts.append((left_x, y_mid))
            if right_found:
                right_x = int(np.mean(px_r[1])) + xr1
                right_pts.append((right_x, y_mid))

            # Center point chỉ khi CẢ HAI lane detect trong CÙNG window (cùng Y)
            if left_found and right_found:
                center_x = (left_x + right_x) // 2
                center_pts.append((center_x, y_mid))

        return left_pts, right_pts, center_pts


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

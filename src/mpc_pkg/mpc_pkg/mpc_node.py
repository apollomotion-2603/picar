#!/usr/bin/env python3
"""
NMPC Lane Following Node — 6-state Spatial Bicycle Model (Kloeser 2020)

State:   x = [s, n, alpha, v, D, delta]
Control: u = [derD, derDelta]
Solver:  acados RTI + HPIPM
Params:  đọc từ config/nmpc.yaml

Based on Nhã's racecar_nmpc_node.py, integrated with Phi's
perception pipeline (LaneState), EKF, and safety features.
"""

import os, time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from lane_msgs.msg import LaneState, VehicleState

from .acados_settings import acados_settings


class NMPCNode(Node):

    def __init__(self):
        super().__init__('nmpc_node')

        # ── Declare params (default = fallback nếu không có yaml) ──────
        # Vehicle geometry
        self.declare_parameter('lf', 0.11)
        self.declare_parameter('lr', 0.11)

        # Traction force model (shared with drive_model / Nhã's bicycle_model)
        self.declare_parameter('mass', 1.5)
        self.declare_parameter('C1', 0.5)
        self.declare_parameter('C2', 15.5)
        self.declare_parameter('Cm1', 0.28)
        self.declare_parameter('Cm2', 0.05)
        self.declare_parameter('Cr0', 0.006)
        self.declare_parameter('Cr2', 0.011)
        self.declare_parameter('cr3', 5.0)

        # Solver
        self.declare_parameter('N', 20)
        self.declare_parameter('Tf', 1.0)
        self.declare_parameter('ctrl_hz', 20)
        self.declare_parameter('timeout', 1.0)

        # Bounds
        self.declare_parameter('delta_max', 0.6109)
        self.declare_parameter('n_max', 0.20)
        self.declare_parameter('throttle_min', -1.0)
        self.declare_parameter('throttle_max', 1.0)
        self.declare_parameter('ddelta_max', 2.0)
        self.declare_parameter('dthrottle_max', 10.0)
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('alat_max', 4.0)
        self.declare_parameter('along_max', 4.0)

        # κ safety
        self.declare_parameter('kappa_max', 2.0)

        # Speed adaptation
        self.declare_parameter('kappa_speed_factor', 8.0)
        self.declare_parameter('v_min_curve', 0.25)

        # κ EMA smoothing (0 = no smoothing, 1 = freeze). Per-stage on κ(s_j)
        # với s_j = j·s_max/N (arc-length uniform, v-independent).
        self.declare_parameter('kappa_ema_alpha', 0.7)

        # EKF
        self.declare_parameter('ekf_timeout', 1.0)

        # Solver build dir
        self.declare_parameter('solver_build_dir', 'nmpc_build_6state')

        # Target speed for cost reference
        self.declare_parameter('target_v', 0.6)

        # ── Load params ────────────────────────────────────────────────
        self.LF         = self.get_parameter('lf').value
        self.LR         = self.get_parameter('lr').value
        self.MASS       = self.get_parameter('mass').value
        self.C1         = self.get_parameter('C1').value
        self.C2         = self.get_parameter('C2').value
        self.CM1        = self.get_parameter('Cm1').value
        self.CM2        = self.get_parameter('Cm2').value
        self.CR0        = self.get_parameter('Cr0').value
        self.CR2        = self.get_parameter('Cr2').value
        self.CR3        = self.get_parameter('cr3').value
        self.N          = self.get_parameter('N').value
        self.TF         = self.get_parameter('Tf').value
        self.CTRL_HZ    = self.get_parameter('ctrl_hz').value
        self.TIMEOUT    = self.get_parameter('timeout').value
        self.DELTA_MAX  = self.get_parameter('delta_max').value
        self.N_MAX      = self.get_parameter('n_max').value
        self.THROT_MIN  = self.get_parameter('throttle_min').value
        self.THROT_MAX  = self.get_parameter('throttle_max').value
        self.DDELTA_MAX = self.get_parameter('ddelta_max').value
        self.DTHROT_MAX = self.get_parameter('dthrottle_max').value
        self.V_MAX      = self.get_parameter('v_max').value
        self.V_MIN      = self.get_parameter('v_min').value
        self.ALAT_MAX   = self.get_parameter('alat_max').value
        self.ALONG_MAX  = self.get_parameter('along_max').value
        self.KAPPA_MAX  = self.get_parameter('kappa_max').value
        self.KAPPA_FACTOR    = self.get_parameter('kappa_speed_factor').value
        self.V_MIN_CURVE     = self.get_parameter('v_min_curve').value
        self.KAPPA_EMA_ALPHA = self.get_parameter('kappa_ema_alpha').value
        self.EKF_TIMEOUT     = self.get_parameter('ekf_timeout').value
        solver_build_dir     = self.get_parameter('solver_build_dir').value
        self.TARGET_V        = self.get_parameter('target_v').value

        # ── Build solver ───────────────────────────────────────────────
        ws = os.path.expanduser(
            '~/main/1_projects/1_autonomous_car_research/ros2_ws')
        build_dir = os.path.join(ws, solver_build_dir)

        self.get_logger().info(
            f'Building 6-state NMPC solver | N={self.N} Tf={self.TF}s ...')
        self.constraint, self.model_ns, self.solver = acados_settings(
            self.TF, self.N, build_dir,
            m=self.MASS, C1=self.C1, C2=self.C2,
            Cm1=self.CM1, Cm2=self.CM2,
            Cr0=self.CR0, Cr2=self.CR2, cr3=self.CR3,
            n_max=self.N_MAX, delta_max=self.DELTA_MAX,
            throttle_min=self.THROT_MIN, throttle_max=self.THROT_MAX,
            ddelta_max=self.DDELTA_MAX, dthrottle_max=self.DTHROT_MAX,
            alat_max=self.ALAT_MAX, along_max=self.ALONG_MAX,
        )
        self.get_logger().info('✅ 6-state NMPC solver ready')

        # ── State [s, n, alpha, v, D, delta] ───────────────────────────
        self.current_state = np.zeros(6)

        # Per-stage κ EMA state — lấy mẫu κ(s_j) trên cubic tại các điểm
        # arc-length uniform (v-independent), nên stage j giữa các frames
        # ứng với cùng 1 điểm vật lý → EMA có ý nghĩa + giữ preview 1s.
        self.kappa_horizon_smooth = None

        # ── Lane perception data ───────────────────────────────────────
        self.lane_coeffs  = None   # [a, b, c, d] cubic polynomial
        self.lane_s_max   = 0.5    # max valid arc-length
        self.kappa_raw    = 0.0    # raw kappa at s=0
        self.lane_ok      = False
        self.last_lane_time = self.get_clock().now()

        # ── Perception fallback (raw values before EKF healthy) ────────
        self.perc_n     = 0.0
        self.perc_alpha = 0.0

        # ── EKF ────────────────────────────────────────────────────────
        self.ekf_v       = None
        self.ekf_n       = None
        self.ekf_alpha   = None
        self.ekf_healthy = False

        # ── car_enabled ────────────────────────────────────────────────
        self.car_enabled = False

        # ── Warm start solver ──────────────────────────────────────────
        self._warm_start()

        # ── Publishers ─────────────────────────────────────────────────
        self.pub_delta = self.create_publisher(Float64, '/steering_angle', 10)
        self.pub_vel   = self.create_publisher(Float64, '/velocity', 10)

        # ── Subscribers ────────────────────────────────────────────────
        self.sub_lane = self.create_subscription(
            LaneState, '/perception/lane_state', self.lane_cb, 10)
        self.sub_ekf = self.create_subscription(
            VehicleState, '/ekf/vehicle_state', self.ekf_cb, 10)
        self.sub_enabled = self.create_subscription(
            Bool, '/car_enabled', self.enabled_cb, 10)

        # ── Timer ──────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.CTRL_HZ, self.control_cb)

        self.get_logger().info(
            f'NMPC 6-state running @ {self.CTRL_HZ}Hz, '
            f'horizon={self.TF}s/{self.N}steps, target_v={self.TARGET_V}')

    # ── Lane state callback ──────────────────────────────────────────────
    def lane_cb(self, msg: LaneState):
        if not msg.lane_detected:
            return
        self.perc_n     = msg.e_y
        self.perc_alpha = msg.e_psi
        self.kappa_raw  = msg.kappa

        # Store cubic coefficients for κ(s) interpolation
        self.lane_coeffs = np.array([
            msg.coeff_a, msg.coeff_b, msg.coeff_c, msg.coeff_d])
        self.lane_s_max = msg.s_max

        self.lane_ok = True
        self.last_lane_time = self.get_clock().now()

    # ── EKF callback ─────────────────────────────────────────────────────
    def ekf_cb(self, msg: VehicleState):
        if msg.ekf_healthy:
            self.ekf_v       = msg.v
            self.ekf_n       = msg.n
            self.ekf_alpha   = msg.alpha
            self.ekf_healthy = True
        else:
            self.ekf_healthy = False

    # ── car_enabled callback ─────────────────────────────────────────────
    def enabled_cb(self, msg: Bool):
        prev = self.car_enabled
        self.car_enabled = msg.data
        if not prev and self.car_enabled:
            # Reset state on False→True
            self.current_state = np.zeros(6)
            if self.ekf_healthy and self.ekf_v is not None:
                self.current_state[3] = self.ekf_v
            self.kappa_raw = 0.0
            self.lane_coeffs = None
            self.kappa_horizon_smooth = None
            self._warm_start()
            self.get_logger().info(
                'car_enabled: False→True, reset 6-state + solver')

    # ── κ(s_j) per-stage, arc-length uniform ─────────────────────────────
    def _compute_kappa_horizon(self) -> np.ndarray:
        """
        Per-stage κ(s_j) với s_j = j · s_max / N (arc-length uniform).

        Khác bản cũ: s_j không còn phụ thuộc v (`v·j·dt`) nên stage j giữa
        các frames ứng với cùng 1 điểm vật lý trên cubic → EMA per-stage
        hợp lệ. Đồng bộ với `s_horizon` dùng cho yref (line bên dưới).

        κ(s) = x_c''(s) / (1 + x_c'(s)²)^(3/2) với cubic x_c(s)=a·s³+b·s²+c·s+d.
        """
        if self.lane_coeffs is None:
            return np.zeros(self.N + 1)

        a, b, c, _d = self.lane_coeffs
        s_max = max(float(self.lane_s_max), 0.05)
        ds = s_max / self.N
        kappa_arr = np.zeros(self.N + 1)

        for j in range(self.N + 1):
            s_j = j * ds
            xp  = 3*a*s_j**2 + 2*b*s_j + c
            xpp = 6*a*s_j + 2*b
            denom = (1 + xp**2)**1.5
            if abs(denom) > 1e-8:
                kappa_arr[j] = np.clip(
                    xpp / denom, -self.KAPPA_MAX, self.KAPPA_MAX)

        a_ema = self.KAPPA_EMA_ALPHA
        if self.kappa_horizon_smooth is None:
            self.kappa_horizon_smooth = kappa_arr.copy()
        else:
            self.kappa_horizon_smooth = (
                a_ema * self.kappa_horizon_smooth + (1.0 - a_ema) * kappa_arr)

        return self.kappa_horizon_smooth.copy()

    # ── Adaptive v_ref per horizon stage ─────────────────────────────────
    def _compute_v_ref_horizon(self, kappa_arr: np.ndarray) -> np.ndarray:
        """
        v_ref(κ_j) = target_v / (1 + kappa_factor·|κ_j|), clipped to
        [v_min_curve, v_max]. Cho phép solver "nhìn trước" khúc cua và
        giảm tốc sớm thông qua optimizing D.
        """
        v_ref = self.TARGET_V / (1.0 + self.KAPPA_FACTOR * np.abs(kappa_arr))
        return np.clip(v_ref, self.V_MIN_CURVE, self.V_MAX)

    # ── Control loop ─────────────────────────────────────────────────────
    def control_cb(self):
        if not self.car_enabled:
            self._publish(0.0, 0.0)
            return

        dt_lane = (self.get_clock().now()
                   - self.last_lane_time).nanoseconds * 1e-9
        if dt_lane > self.TIMEOUT or not self.lane_ok:
            self._publish(0.0, 0.0)
            return

        # ── Build initial state x0 ────────────────────────────────────
        x0 = self.current_state.copy()
        x0[0] = 0.0   # s always relative (reset each frame)

        # n, alpha: prefer EKF if healthy
        if self.ekf_healthy and self.ekf_n is not None:
            x0[1] = float(np.clip(self.ekf_n, -self.N_MAX, self.N_MAX))
            x0[2] = float(np.clip(self.ekf_alpha,
                                  -self.DELTA_MAX, self.DELTA_MAX))
        else:
            x0[1] = self.perc_n
            x0[2] = self.perc_alpha

        # v: prefer EKF if healthy
        if self.ekf_healthy and self.ekf_v is not None:
            x0[3] = float(np.clip(self.ekf_v, self.V_MIN, self.V_MAX))

        # D, delta: warm-start from previous solver prediction
        # (already in self.current_state[4], [5])

        # ── Set initial state constraint ──────────────────────────────
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        # ── κ interpolation + adaptive v_ref per-stage ────────────────
        kappa_horizon = self._compute_kappa_horizon()
        v_ref_horizon = self._compute_v_ref_horizon(kappa_horizon)

        # ── Build s reference trajectory (relative, 0 → s_max) ────────
        s_max = float(self.lane_s_max) if self.lane_s_max > 0.05 else 0.5
        s_horizon = np.linspace(0.0, s_max, self.N + 1)

        # ── Set per-stage parameters and references ───────────────────
        for j in range(self.N):
            self.solver.set(j, "p", np.array([kappa_horizon[j]]))
            # yref = [s, n_ref, alpha_ref, v_ref, D_ref, delta_ref,
            #         derD_ref, dDelta_ref]
            yref = np.array([
                s_horizon[j], 0.0, 0.0,
                v_ref_horizon[j], 0.0, 0.0, 0.0, 0.0])
            self.solver.set(j, "yref", yref)

        # Terminal stage
        self.solver.set(self.N, "p",
                        np.array([kappa_horizon[self.N]]))
        yref_N = np.array([
            s_horizon[-1], 0.0, 0.0,
            v_ref_horizon[self.N], 0.0, 0.0])
        self.solver.set(self.N, "yref", yref_N)

        # ── Solve ─────────────────────────────────────────────────────
        t0 = time.time()
        status = self.solver.solve()
        t_ms = (time.time() - t0) * 1000

        if status != 0:
            self.get_logger().warn(
                f'NMPC solver status={status}, hold previous command')
            self._publish(
                float(self.current_state[5]),  # delta from prev
                0.0)
            return

        # ── Extract control from solver's stage-1 prediction ──────────
        x1 = self.solver.get(1, "x")
        D_cmd     = float(x1[4])
        delta_cmd = float(x1[5])
        # Trust solver's v: output its predicted velocity at stage 1
        # (clip to non-negative — vehicle_controller treats /velocity
        # as a signed setpoint and we don't want reverse for lane follow)
        v_cmd = float(np.clip(x1[3], 0.0, self.V_MAX))

        # Update current_state with solver prediction for warm-start
        self.current_state = x1.copy()

        self._publish(delta_cmd, v_cmd)

        # ── Logging every frame (verbose during debug) ────────────────
        v_src = 'EKF' if (self.ekf_healthy
                          and self.ekf_v is not None) else 'perc'
        self.get_logger().info(
            f'n={x0[1]*1000:.1f}mm '
            f'α={np.degrees(x0[2]):.1f}° '
            f'v={x0[3]:.2f}({v_src}) '
            f'D={D_cmd:.3f} '
            f'δ={np.degrees(delta_cmd):.1f}° '
            f'κ₀={kappa_horizon[0]:.4f} '
            f'vref₀={v_ref_horizon[0]:.2f} '
            f'v→{v_cmd:.2f} '
            f'{t_ms:.1f}ms '
            f'st={status}')

    # ── Warm start solver ────────────────────────────────────────────────
    def _warm_start(self):
        x0 = self.current_state.copy()
        for i in range(self.N + 1):
            self.solver.set(i, 'x', x0)
            self.solver.set(i, 'p', np.array([0.0]))
        for i in range(self.N):
            self.solver.set(i, 'u', np.array([0.0, 0.0]))

    # ── Publish ──────────────────────────────────────────────────────────
    def _publish(self, delta: float, v: float):
        msg_d = Float64(); msg_d.data = delta
        msg_v = Float64(); msg_v.data = v
        self.pub_delta.publish(msg_d)
        self.pub_vel.publish(msg_v)


def main(args=None):
    rclpy.init(args=args)
    node = NMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
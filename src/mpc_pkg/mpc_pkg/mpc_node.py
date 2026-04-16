#!/usr/bin/env python3
"""
NMPC Lane Following Node — Kloeser 2020
State:   x = [s, n, alpha, v]
Control: u = [delta, v_ref]
Solver:  acados RTI + HPIPM
Params:  đọc từ config/nmpc.yaml
"""

import os, time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from lane_msgs.msg import LaneState, VehicleState

os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import casadi as ca


def build_solver(build_dir, LF, LR, TAU_V, N, TF,
                 W_N, W_ALPHA, W_DELTA, W_V, W_S_E, W_N_E, W_A_E,
                 DELTA_MAX, V_MAX, V_MIN, N_MAX) -> AcadosOcpSolver:
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)
    L = LF + LR

    model = AcadosModel()
    model.name = 'rc_car_kloeser'

    s     = ca.SX.sym('s')
    n     = ca.SX.sym('n')
    alpha = ca.SX.sym('alpha')
    v     = ca.SX.sym('v')
    x     = ca.vertcat(s, n, alpha, v)

    delta   = ca.SX.sym('delta')
    v_ref   = ca.SX.sym('v_ref')
    u       = ca.vertcat(delta, v_ref)
    kappa_c = ca.SX.sym('kappa_c')

    beta      = (LR / L) * delta
    s_dot     = v * ca.cos(alpha + beta) / (1.0 - n * kappa_c)
    n_dot     = v * ca.sin(alpha + beta)
    alpha_dot = (v / LR) * ca.sin(beta) - kappa_c * s_dot
    v_dot     = (v_ref - v) / TAU_V

    model.x           = x
    model.u           = u
    model.p           = kappa_c
    model.f_expl_expr = ca.vertcat(s_dot, n_dot, alpha_dot, v_dot)
    model.xdot        = ca.SX.sym('xdot', x.shape)

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon       = N
    ocp.solver_options.tf              = TF
    ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx  = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.print_level     = 0

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = (
        W_N     * n**2
        + W_ALPHA * alpha**2
        + W_DELTA * delta**2
        + W_V     * (v - v_ref)**2
    )

    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_e = (
        -W_S_E * s
        + W_N_E * n**2
        + W_A_E * alpha**2
    )

    ocp.constraints.lbx   = np.array([-N_MAX,    -DELTA_MAX, V_MIN])
    ocp.constraints.ubx   = np.array([ N_MAX,     DELTA_MAX, V_MAX])
    ocp.constraints.idxbx = np.array([1, 2, 3])

    ocp.constraints.lbu   = np.array([-DELTA_MAX, V_MIN])
    ocp.constraints.ubu   = np.array([ DELTA_MAX, V_MAX])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.3])
    ocp.parameter_values = np.array([0.0])

    solver = AcadosOcpSolver(ocp, json_file='rc_car_nmpc.json')
    return solver


class NMPCNode(Node):

    def __init__(self):
        super().__init__('nmpc_node')

        # ── Declare params (default = fallback nếu không có yaml) ──
        self.declare_parameter('lf', 0.11)
        self.declare_parameter('lr', 0.11)
        self.declare_parameter('tau_v', 0.1)
        self.declare_parameter('N', 30)
        self.declare_parameter('Tf', 1.0)
        self.declare_parameter('ctrl_hz', 30)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('w_n', 200.0)
        self.declare_parameter('w_alpha', 100.0)
        self.declare_parameter('w_delta', 10.0)
        self.declare_parameter('w_v', 10.0)
        self.declare_parameter('w_s_e', 100.0)
        self.declare_parameter('w_n_e', 1000.0)
        self.declare_parameter('w_a_e', 5000.0)
        self.declare_parameter('delta_max', 0.6109)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('n_max', 0.20)
        self.declare_parameter('kappa_speed_factor', 8.0)
        self.declare_parameter('v_min_curve', 0.2)
        self.declare_parameter('ekf_timeout', 1.0)

        # ── Load params ──
        self.LF    = self.get_parameter('lf').value
        self.LR    = self.get_parameter('lr').value
        self.L     = self.LF + self.LR
        self.TAU_V = self.get_parameter('tau_v').value
        self.N     = self.get_parameter('N').value
        self.TF    = self.get_parameter('Tf').value
        self.CTRL_HZ      = self.get_parameter('ctrl_hz').value
        self.TIMEOUT      = self.get_parameter('timeout').value
        self.W_N          = self.get_parameter('w_n').value
        self.W_ALPHA      = self.get_parameter('w_alpha').value
        self.W_DELTA      = self.get_parameter('w_delta').value
        self.W_V          = self.get_parameter('w_v').value
        self.W_S_E        = self.get_parameter('w_s_e').value
        self.W_N_E        = self.get_parameter('w_n_e').value
        self.W_A_E        = self.get_parameter('w_a_e').value
        self.DELTA_MAX    = self.get_parameter('delta_max').value
        self.V_MAX        = self.get_parameter('v_max').value
        self.V_MIN        = self.get_parameter('v_min').value
        self.N_MAX        = self.get_parameter('n_max').value
        self.KAPPA_FACTOR = self.get_parameter('kappa_speed_factor').value
        self.V_MIN_CURVE  = self.get_parameter('v_min_curve').value
        self.EKF_TIMEOUT  = self.get_parameter('ekf_timeout').value

        # ── Build solver ──
        ws = os.path.expanduser(
            '~/main/1_projects/1_autonomous_car_research/ros2_ws')
        build_dir = os.path.join(ws, 'nmpc_build')

        self.get_logger().info(
            f'Building NMPC solver | V_MAX={self.V_MAX} N={self.N} Tf={self.TF}s ...')
        self.solver = build_solver(
            build_dir,
            self.LF, self.LR, self.TAU_V,
            self.N, self.TF,
            self.W_N, self.W_ALPHA, self.W_DELTA, self.W_V,
            self.W_S_E, self.W_N_E, self.W_A_E,
            self.DELTA_MAX, self.V_MAX, self.V_MIN, self.N_MAX)
        self.get_logger().info('✅ NMPC solver ready')

        # ── State ──
        self.x_est          = np.array([0.0, 0.0, 0.0, 0.3])
        self.u_prev         = np.array([0.0, 0.3])
        self.kappa          = 0.0
        self.v_ref_adaptive = 0.3
        self.lane_ok        = False
        self.last_lane_time = self.get_clock().now()

        # ── EKF ──
        self.ekf_v       = None
        self.ekf_n       = None
        self.ekf_alpha   = None
        self.ekf_healthy = False

        # Raw perception fallback (dùng khi EKF chưa healthy)
        self.perc_n     = 0.0
        self.perc_alpha = 0.0

        self.poly_a = 0.0
        self.poly_b = 0.0
        self.poly_c = 0.0
        self.poly_d = 0.0
        self.perc_s_max = 0.7  # camera lookahead [m]; update từ LaneState.s_max

        self._warm_start(self.x_est)

        # ── Publishers ──
        self.pub_delta = self.create_publisher(Float64, '/steering_angle', 10)
        self.pub_vel   = self.create_publisher(Float64, '/velocity', 10)

        # ── Subscribers ──
        self.sub_lane = self.create_subscription(
            LaneState, '/perception/lane_state', self.lane_cb, 10)
        self.sub_ekf = self.create_subscription(
            VehicleState, '/ekf/vehicle_state', self.ekf_cb, 10)
        self.car_enabled = False
        self.sub_enabled = self.create_subscription(
            Bool, '/car_enabled', self.enabled_cb, 10)

        # ── Timer ──
        self.timer = self.create_timer(1.0 / self.CTRL_HZ, self.control_cb)

        self.get_logger().info(
            f'NMPC node running @ {self.CTRL_HZ}Hz, horizon={self.TF}s/{self.N}steps')

    def lane_cb(self, msg: LaneState):
        if not msg.lane_detected:
            return
        self.perc_n     = msg.e_y
        self.perc_alpha = msg.e_psi
        self.kappa      = msg.kappa
        self.lane_ok   = True
        self.last_lane_time = self.get_clock().now()
        
        self.poly_a = msg.coeff_a
        self.poly_b = msg.coeff_b
        self.poly_c = msg.coeff_c
        self.poly_d = msg.coeff_d
        # Lưu lookahead thực tế từ camera (dùng để clip kappa predict trong horizon)
        if msg.s_max > 0.05:
            self.perc_s_max = msg.s_max

        kappa_abs = abs(msg.kappa)
        self.v_ref_adaptive = float(np.clip(
            self.V_MAX / (1.0 + self.KAPPA_FACTOR * kappa_abs),
            self.V_MIN_CURVE, self.V_MAX))

    def ekf_cb(self, msg: VehicleState):
        if msg.ekf_healthy:
            self.ekf_v       = msg.v
            self.ekf_n       = msg.n
            self.ekf_alpha   = msg.alpha
            self.ekf_healthy = True
        else:
            self.ekf_healthy = False

    def enabled_cb(self, msg: Bool):
        prev = self.car_enabled
        self.car_enabled = msg.data
        # Khi xe được start lại (False→True): reset velocity về giá trị thực tế
        if not prev and self.car_enabled:
            if self.ekf_healthy and self.ekf_v is not None:
                self.x_est[3] = self.ekf_v
            else:
                self.x_est[3] = 0.0  # Xe đang đứng yên → khởi đầu v=0
            self.x_est[0] = 0.0  # Reset progressive s về 0
            self.get_logger().info('car_enabled: False→True, reset x_est về EKF/0')

    def control_cb(self):
        if not self.car_enabled:
            self._publish(0.0, 0.0)
            return

        dt_lane = (self.get_clock().now()
                   - self.last_lane_time).nanoseconds * 1e-9
        if dt_lane > self.TIMEOUT or not self.lane_ok:
            self._publish(0.0, 0.0)
            return

        # Dùng EKF n/alpha (đã lọc) nếu healthy, fallback sang raw perception
        if self.ekf_healthy and self.ekf_n is not None:
            self.x_est[1] = float(np.clip(self.ekf_n,     -self.N_MAX,     self.N_MAX))
            self.x_est[2] = float(np.clip(self.ekf_alpha, -self.DELTA_MAX, self.DELTA_MAX))
        else:
            self.x_est[1] = self.perc_n
            self.x_est[2] = self.perc_alpha

        self.solver.set(0, 'lbx', self.x_est)
        self.solver.set(0, 'ubx', self.x_est)

        dt_step = self.TF / self.N
        s_max_cam = self.perc_s_max * 0.9  # 90% số thực để an toàn, tránh extrapolate
        for i in range(self.N):
            # Clip s_pred trong phạm vi camera thấy: tránh cubic poly extrapolate vô tội vạ
            s_pred = min(max(self.x_est[3], 0.1) * i * dt_step, s_max_cam)
            xp  = 3.0 * self.poly_a * s_pred**2 + 2.0 * self.poly_b * s_pred + self.poly_c
            xpp = 6.0 * self.poly_a * s_pred + 2.0 * self.poly_b
            kappa_pred = xpp / max((1.0 + xp**2)**1.5, 1e-6)
            
            self.solver.set(i, 'p', np.array([kappa_pred]))
            
            # Dynamic predictive braking bound
            v_ref_i = float(np.clip(
                self.V_MAX / (1.0 + self.KAPPA_FACTOR * abs(kappa_pred)),
                self.V_MIN_CURVE, self.V_MAX))
            self.solver.constraints_set(i, 'ubu', np.array([self.DELTA_MAX, v_ref_i]))

        # Terminal node parameter — clip tương tự
        s_end = min(max(self.x_est[3], 0.1) * self.N * dt_step, s_max_cam)
        xp_end  = 3.0 * self.poly_a * s_end**2 + 2.0 * self.poly_b * s_end + self.poly_c
        xpp_end = 6.0 * self.poly_a * s_end + 2.0 * self.poly_b
        kappa_end = xpp_end / max((1.0 + xp_end**2)**1.5, 1e-6)
        self.solver.set(self.N, 'p', np.array([kappa_end]))

        t0     = time.time()
        status = self.solver.solve()
        t_ms   = (time.time() - t0) * 1000

        if status not in [0, 2]:
            self.get_logger().warn(
                f'NMPC solver status={status}, using prev u')
            delta_cmd = float(self.u_prev[0])
            v_cmd     = float(self.u_prev[1])
        else:
            u_opt     = self.solver.get(0, 'u')
            delta_cmd = float(np.clip(u_opt[0], -self.DELTA_MAX, self.DELTA_MAX))
            v_cmd     = float(np.clip(u_opt[1], self.V_MIN, self.v_ref_adaptive))

        self.u_prev = np.array([delta_cmd, v_cmd])

        dt     = 1.0 / self.CTRL_HZ
        beta   = (self.LR / self.L) * delta_cmd
        kc     = self.kappa
        n_     = self.x_est[1]
        alpha_ = self.x_est[2]
        v_     = self.x_est[3]
        denom  = max(1.0 - n_ * kc, 0.1)
        s_dot  = v_ * np.cos(alpha_ + beta) / denom
        self.x_est[0] += s_dot * dt

        if self.ekf_healthy and self.ekf_v is not None:
            self.x_est[3] = float(np.clip(self.ekf_v, self.V_MIN, self.V_MAX))
        else:
            self.x_est[3] += (v_cmd - v_) / self.TAU_V * dt
            self.x_est[3]  = float(np.clip(self.x_est[3], self.V_MIN, self.V_MAX))

        self._publish(delta_cmd, v_cmd)

        if int(time.time() * 5) % 5 == 0:
            v_src = 'EKF' if (self.ekf_healthy and self.ekf_v is not None) else 'model'
            self.get_logger().info(
                f'n={self.x_est[1]*1000:.1f}mm '
                f'α={np.degrees(self.x_est[2]):.2f}° '
                f'v={self.x_est[3]:.2f}m/s({v_src}) '
                f'δ={np.degrees(delta_cmd):.2f}° '
                f'κ={self.kappa:.3f} '
                f'v_cmd={v_cmd:.2f} '
                f't={t_ms:.1f}ms')

    def _warm_start(self, x0):
        for i in range(self.N + 1):
            self.solver.set(i, 'x', x0)
            self.solver.set(i, 'p', np.array([0.0]))
        for i in range(self.N):
            self.solver.set(i, 'u', np.array([0.0, x0[3]]))

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
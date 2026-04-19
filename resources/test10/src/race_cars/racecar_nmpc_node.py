import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np

from acados_settings import acados_settings


class RacecarNMPCNode(Node):
    def __init__(self):
        super().__init__('racecar_nmpc_node')

        # ── NMPC parameters ───────────────────────────────────────────
        self.Tf       = 1.0    # prediction horizon [s]
        self.N        = 20     # discretisation steps
        self.target_v = 0.6   # reference speed [m/s]  (tune this!)

        # Build acados solver (no track file needed)
        self.constraint, self.model, self.acados_solver = \
            acados_settings(self.Tf, self.N)

        # ── State: [s, n, alpha, v, D, delta] ─────────────────────────
        # s     = 0  (relative, reset each frame)
        # n     ← /line/cte
        # alpha ← /line/heading_angle
        # v     ← /odom
        # D     warm-started from previous predicted state
        # delta warm-started from previous predicted state
        self.current_state = np.zeros(6)

        # ── Data-ready flags ──────────────────────────────────────────
        self.s_ready     = False
        self.kappa_ready = False
        self.s_ref       = None   # Float64MultiArray data array
        self.kappa_ref   = None   # Float64MultiArray data array

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/ground_truth/odom/filtered', self.odom_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/line/s_metric', self.s_ref_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/line/kappa_s', self.kappa_ref_callback, 10)
        self.create_subscription(
            Float64, '/line/cte', self.cte_callback, 10)
        self.create_subscription(
            Float64, '/line/heading_angle', self.heading_angle_callback, 10)

        # ── Publishers — matches vehicle_controller interface ──────────
        # vehicle_controller subscribes /velocity (Float64) and
        # /steering_angle (Float64), NOT /cmd_vel (Twist).
        self.vel_pub   = self.create_publisher(Float64, '/velocity', 10)
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.throttle_pub = self.create_publisher(Float64, '/throttle', 10)

        # ── Control timer: 20 Hz ──────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

    # ── Callbacks ─────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """Update longitudinal velocity from odometry."""
        self.current_state[3] = msg.twist.twist.linear.x   # v [m/s]

    def s_ref_callback(self, msg: Float64MultiArray):
        """Receive arc-length array [0, s1, ..., sN_img] from image_processor."""
        self.s_ref   = np.array(msg.data)
        self.s_ready = True

    def kappa_ref_callback(self, msg: Float64MultiArray):
        """Receive curvature array [k0, ..., kN_img] from image_processor."""
        self.kappa_ref   = np.array(msg.data)
        self.kappa_ready = True

    def cte_callback(self, msg: Float64):
        """Update cross-track error n [m] from image_processor."""
        self.current_state[1] = msg.data

    def heading_angle_callback(self, msg: Float64):
        """Update heading error alpha [rad] from image_processor."""
        self.current_state[2] = msg.data

    # ── Control loop ──────────────────────────────────────────────────

    def control_loop(self):
        """20 Hz NMPC control loop."""
        if not (self.s_ready and self.kappa_ready):
            return   # wait until image_processor data arrives

        # Build initial condition (s is always 0 — relative frame)
        x0 = self.current_state.copy()
        x0[0] = 0.0

        # ── Set initial state constraint ───────────────────────────────
        self.acados_solver.set(0, "lbx", x0)
        self.acados_solver.set(0, "ubx", x0)

        # ── Interpolate kappa over the N+1 horizon stages ─────────────
        # kappa_ref has N_img (=5) points; interpolate to N+1 = 6 points
        kappa_horizon = np.interp(
            np.linspace(0, 1, self.N + 1),
            np.linspace(0, 1, len(self.kappa_ref)),
            self.kappa_ref,
        )

        # ── Build s reference trajectory (relative, 0 → s_max) ────────
        s_max     = float(self.s_ref[-1]) if len(self.s_ref) > 0 else 0.5
        s_horizon = np.linspace(0.0, s_max, self.N + 1)

        # ── Set per-stage parameters and references ────────────────────
        for j in range(self.N):
            # kappa_p: curvature at this horizon stage [1/m]
            self.acados_solver.set(j, "p", np.array([kappa_horizon[j]]))
            # yref = [s, n_ref, alpha_ref, v_ref, D_ref, delta_ref, derD_ref, dDelta_ref]
            # yref = np.array([s_horizon[j], 0.0, 0.0,
            #                  self.target_v, 0.0, 0.0, 0.0, 0.0])
            yref = np.array([s_horizon[j], 0.0, 0.0,
                             0, 0.0, 0.0, 0.0, 0.0])
            self.acados_solver.set(j, "yref", yref)

        # Terminal stage
        self.acados_solver.set(self.N, "p",
                               np.array([kappa_horizon[self.N]]))
        # yref_N = np.array([s_horizon[-1], 0.0, 0.0,
        #                    self.target_v, 0.0, 0.0])
        yref_N = np.array([s_horizon[-1], 0.0, 0.0,
                           0, 0.0, 0.0])
        self.acados_solver.set(self.N, "yref", yref_N)

        # ── Solve ──────────────────────────────────────────────────────
        status = self.acados_solver.solve()
        if status != 0:
            self.get_logger().error(f"[NMPC] acados returned status {status} — Solver failed, no control command published!")
            # Optionally: add more diagnostics or error handling here
            return

        # ── Extract steering from NMPC (predicted state at step 1) ─────
        # NOTE: We do NOT use x1[3] (predicted velocity) as the velocity
        # command because at cold start v≈0, making x1[3]≈0 and the car
        # never moves.  Instead, we send target_v directly (/velocity is
        # a setpoint for vehicle_controller) and let NMPC handle steering.
        x1        = self.acados_solver.get(1, "x")
        # Publish NMPC control commands: duty cycle (D) and steering angle (delta)
        D_cmd = float(x1[4])
        delta_cmd = float(x1[5])
        
        throttle_msg = Float64()
        throttle_msg.data = D_cmd
        self.throttle_pub.publish(throttle_msg)
        
        steer_msg = Float64()
        steer_msg.data = delta_cmd
        self.steer_pub.publish(steer_msg)
        
        self.get_logger().info(
            f"D_cmd={throttle_msg.data:.3f} | δ={float(np.degrees(delta_cmd)):.1f}° | "
            f"n={x0[1]:.3f} m | α={float(np.degrees(x0[2])):.1f}°"
        )
        # (Optional) Still publish velocity for debug if needed
        # vel_msg = Float64()
        # vel_msg.data = float(x1[3])
        # self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RacecarNMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
EKF Node — Tuần 5
Fuse Camera (30Hz) + IMU (100Hz) + JointStates (100Hz) → VehicleState (100Hz)

Subscribe:
  /imu/data              sensor_msgs/Imu         100Hz  (Bước C: Gazebo IMU plugin)
  /perception/lane_state lane_msgs/LaneState      30Hz
  /joint_states          sensor_msgs/JointState   100Hz  (đã có từ JointStatePublisher)

Publish:
  /ekf/vehicle_state     lane_msgs/VehicleState   100Hz
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from lane_msgs.msg import LaneState, VehicleState

# ─────────────────────────────────────────
#  Vehicle params
# ─────────────────────────────────────────
WHEEL_RADIUS = 0.04   # m — khớp với parameters.yaml
LF = 0.11
LR = 0.11
L  = 0.22

# ─────────────────────────────────────────


# ─────────────────────────────────────────
#  EKF
# ─────────────────────────────────────────
class RCCarEKF:
    """
    State: x = [n, n_dot, alpha, alpha_dot, v, psi]
    """

    def __init__(self):
        self.x = np.zeros(6)
        self.x[4] = 0.1   # v initial [m/s]

        self.P = np.diag([0.01, 0.05, 0.01, 0.05, 0.1, 0.1])

        self.Q = np.diag([1e-4, 1e-3, 1e-4, 1e-3, 5e-4, 1e-4])

        # R camera: [n, alpha]
        self.R_cam = np.diag([(0.01)**2, (0.02)**2])

        # R full: [n, alpha, v]
        self.R_full = np.diag([(0.01)**2, (0.02)**2, (0.02)**2])



        self.H_cam = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ])
        self.H_full = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
        ])

    def predict(self, ax, ay, yaw_rate, kappa, dt):
        n, n_dot, alpha, alpha_dot, v, psi = self.x

        denom = max(1.0 - n * kappa, 0.1)
        s_dot = v * np.cos(alpha) / denom

        x_new = np.array([
            n + n_dot * dt,
            n_dot + ay * dt,
            alpha + alpha_dot * dt,
            yaw_rate - kappa * s_dot,
            float(np.clip(v + ax * dt, 0.0, 2.0)),
            psi + yaw_rate * dt,
        ])

        d_adot_dn = -kappa * v * np.cos(alpha) * kappa / (denom ** 2)
        d_adot_da =  kappa * v * np.sin(alpha) / denom
        d_adot_dv = -kappa * np.cos(alpha) / denom

        A = np.zeros((6, 6))
        A[0, 1] = 1.0
        A[2, 3] = 1.0
        A[3, 0] = d_adot_dn
        A[3, 2] = d_adot_da
        A[3, 4] = d_adot_dv

        F = np.eye(6) + dt * A
        self.P = F @ self.P @ F.T + self.Q
        self.x = x_new

    def update(self, z, H, R):
        """Generic update step."""
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def update_camera(self, n_meas, alpha_meas):
        z = np.array([n_meas, alpha_meas])
        self.update(z, self.H_cam, self.R_cam)

    def update_camera_and_velocity(self, n_meas, alpha_meas, v_meas):
        z = np.array([n_meas, alpha_meas, v_meas])
        self.update(z, self.H_full, self.R_full)

    def is_healthy(self):
        """Check nếu covariance diverge."""
        diag = np.diag(self.P)
        return bool(np.all(diag < 100.0) and np.all(diag > 0.0))


# ─────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────
class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        self.ekf   = RCCarEKF()
        self.kappa = 0.0


        # IMU state
        self.ax       = 0.0
        self.ay       = 0.0
        self.yaw_rate = 0.0
        self.imu_ok   = False
        self.last_imu_time = self.get_clock().now()

        # Velocity từ joint_states
        self.v_wheel = None

        # Publishers
        self.pub = self.create_publisher(VehicleState, '/ekf/vehicle_state', 10)

        # Subscribers
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(LaneState, '/perception/lane_state', self.lane_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # EKF timer @ 100Hz — chạy predict mỗi 10ms
        self.dt = 0.01
        self.create_timer(self.dt, self.ekf_cb)

        self.get_logger().info('EKF node started @ 100Hz')
        self.get_logger().info('Waiting for /imu/data ... (sẽ có sau Bước C)')

    # ── IMU callback ───────────────────────────────
    def imu_cb(self, msg: Imu):
        self.ax       = msg.linear_acceleration.x
        self.ay       = msg.linear_acceleration.y
        self.yaw_rate = msg.angular_velocity.z
        self.imu_ok   = True
        self.last_imu_time = self.get_clock().now()

    # ── Lane state callback ────────────────────────
    def lane_cb(self, msg: LaneState):
        if not msg.lane_detected:
            return

        # EMA kappa: 0.7 old + 0.3 new — chống noise ảnh hưởng EKF predict alpha_dot
        self.kappa = 0.70 * self.kappa + 0.30 * msg.kappa


        # Update EKF với camera measurement
        if self.v_wheel is not None:
            self.ekf.update_camera_and_velocity(
                msg.e_y, msg.e_psi, self.v_wheel)
        else:
            self.ekf.update_camera(msg.e_y, msg.e_psi)

    # ── JointState callback ────────────────────────
    def joint_cb(self, msg: JointState):
        """
        Tính v từ rear wheel angular velocity.
        joint_states publish: rear_left_wheel_joint, rear_right_wheel_joint
        v = avg(|omega_L|, |omega_R|) * wheel_radius
        """
        vel_map = dict(zip(msg.name, msg.velocity))
        vl = vel_map.get('rear_left_wheel_joint',  None)
        vr = vel_map.get('rear_right_wheel_joint', None)

        if vl is not None and vr is not None:
            omega_avg = (abs(vl) + abs(vr)) / 2.0
            self.v_wheel = omega_avg * WHEEL_RADIUS

    # ── EKF timer @ 100Hz ─────────────────────────
    def ekf_cb(self):
        # Nếu chưa có IMU: dùng zero input (xe đứng yên)
        # EKF vẫn chạy, chỉ propagate uncertainty
        imu_timeout = (self.get_clock().now()
                       - self.last_imu_time).nanoseconds * 1e-9 > 1.0

        if not self.imu_ok or imu_timeout:
            # Dùng zero IMU — predict với ax=ay=yaw_rate=0
            # Đây là fallback trước khi có IMU plugin (Bước C)
            self.ekf.predict(0.0, 0.0, 0.0, self.kappa, self.dt)
        else:
            self.ekf.predict(
                self.ax, self.ay, self.yaw_rate, self.kappa, self.dt)

        # Publish
        msg = VehicleState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body_link'

        x = self.ekf.x
        msg.n         = float(x[0])
        msg.n_dot     = float(x[1])
        msg.alpha     = float(x[2])
        msg.alpha_dot = float(x[3])
        msg.v         = float(x[4])
        msg.psi       = float(x[5])

        msg.std_n     = float(np.sqrt(self.ekf.P[0, 0]))
        msg.std_alpha = float(np.sqrt(self.ekf.P[2, 2]))
        msg.std_v     = float(np.sqrt(self.ekf.P[4, 4]))

        msg.ekf_healthy = self.ekf.is_healthy()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
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
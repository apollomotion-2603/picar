#!/usr/bin/env python3
"""
Reset Node
  Q + Enter : reset ve vi tri ban dau
  S + Enter : xe bat dau chay
  X + Enter : xe dung
"""
import math, threading, subprocess, sys
import rclpy
from rclpy.node import Node
from lane_msgs.msg import LaneState
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger

TIMEOUT = 5.0
MAP_CONFIGS = {
    1: {'world':'lane_track','x':0.0,'y':1.0,'z':0.15,'yaw':0.0},
    2: {'world':'track_test','x':0.0,'y':-2.666,'z':0.15,'yaw':0.0},
    3: {'world':'lane_change','x':0.54,'y':0.75,'z':0.15,'yaw':0.0},
    4: {'world':'obstacle_track','x':0.0,'y':1.0,'z':0.15,'yaw':0.0},
}

def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset_node')
        self.declare_parameter('map_id', 1)
        map_id = self.get_parameter('map_id').value
        if map_id not in MAP_CONFIGS:
            raise SystemExit(1)
        cfg = MAP_CONFIGS[map_id]
        self.world   = cfg['world']
        self.spawn_x = cfg['x']
        self.spawn_y = cfg['y']
        self.spawn_z = cfg['z']
        self.qx,self.qy,self.qz,self.qw = yaw_to_quat(cfg['yaw'])
        self.last_detected_time = self.get_clock().now()
        self.resetting   = False
        self.car_enabled = False
        self._stop_count = 0
        self._stop_timer = None
        self.pub_vel     = self.create_publisher(Float64,'/velocity',10)
        self.pub_delta   = self.create_publisher(Float64,'/steering_angle',10)
        self.pub_enabled = self.create_publisher(Bool,'/car_enabled',10)
        self.create_subscription(LaneState,'/perception/lane_state',self.lane_cb,10)
        self.create_timer(0.5, self.check_timeout)
        self.create_timer(0.1, self.publish_enabled)
        self.create_service(Trigger,'/reset_car',self.reset_srv_cb)
        threading.Thread(target=self._keyboard_listener,daemon=True).start()
        self._publish_stop()
        self.get_logger().info(
            f'Reset node ready — map={map_id} ({self.world}) '
            f'spawn=({self.spawn_x},{self.spawn_y})\n'
            f'  S=START  X=STOP  Q=RESET')

    def publish_enabled(self):
        msg = Bool(); msg.data = self.car_enabled
        self.pub_enabled.publish(msg)

    def lane_cb(self, msg):
        if msg.lane_detected and self.car_enabled:
            self.last_detected_time = self.get_clock().now()
            self.resetting = False

    def check_timeout(self):
        if not self.car_enabled:
            return
        elapsed = (self.get_clock().now()-self.last_detected_time).nanoseconds*1e-9
        if elapsed > TIMEOUT and not self.resetting:
            self.get_logger().warn(f'Lane lost {elapsed:.1f}s — auto reset!')
            self.do_reset()

    def reset_srv_cb(self, request, response):
        self.do_reset()
        response.success = True
        response.message = 'Reset OK'
        return response

    def _keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().info('stdin khong phai TTY — dung service')
            return
        self.get_logger().info('Keyboard: S=start X=stop Q=reset')
        while True:
            try:
                key = sys.stdin.readline().strip().lower()
                if not key: break
                if key == 's':
                    self.car_enabled = True
                    self.last_detected_time = self.get_clock().now()
                    self.get_logger().info('>>> START — xe bat dau chay')
                elif key == 'x':
                    self.car_enabled = False
                    self._publish_stop()
                    self.get_logger().info('>>> STOP — xe dung')
                elif key == 'q':
                    self.get_logger().info('>>> RESET')
                    self.do_reset()
            except Exception as e:
                self.get_logger().error(f'keyboard error: {e}')
                break

    def do_reset(self):
        if self.resetting: return
        self.resetting   = True
        self.car_enabled = False
        self._stop_count = 0
        self._publish_stop()
        if self._stop_timer:
            try: self._stop_timer.cancel()
            except: pass
        self._stop_timer = self.create_timer(0.05, self._stop_publish_timer)
        subprocess.run(['gz','service',
            '-s', f'/world/{self.world}/set_pose',
            '--reqtype','gz.msgs.Pose',
            '--reptype','gz.msgs.Boolean',
            '--timeout','1000',
            '--req',
            f'name: "ackermann_steering_vehicle", '
            f'position: {{x:{self.spawn_x},y:{self.spawn_y},z:{self.spawn_z}}}, '
            f'orientation: {{x:{self.qx},y:{self.qy},z:{self.qz:.4f},w:{self.qw:.4f}}}',
        ], capture_output=True)
        self.get_logger().info(f'Reset -> ({self.spawn_x},{self.spawn_y}) | Nhan S de chay')

    def _publish_stop(self):
        self.pub_vel.publish(Float64(data=0.0))
        self.pub_delta.publish(Float64(data=0.0))

    def _stop_publish_timer(self):
        self._publish_stop()
        self._stop_count += 1
        if self._stop_count >= 20:
            self._stop_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()

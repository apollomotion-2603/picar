#!/usr/bin/env python3
"""
Grid Viewer — hiển thị /perception/debug_grid bằng cv2.imshow()
Chạy: python3 grid_viewer.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GridViewer(Node):
    def __init__(self):
        super().__init__("grid_viewer")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/perception/debug_grid", self.cb, 10)
        cv2.namedWindow("Perception Debug [2x2]", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Perception Debug [2x2]", 1280, 1040)
        self.get_logger().info("Grid viewer ready — chờ /perception/debug_grid")

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Perception Debug [2x2]", img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = GridViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import cv2
import numpy as np

W = 640; H = 480
BEV_W = 400; BEV_H = 500
dst_pts = np.float32([100.0, 0.0, 300.0, 0.0, 300.0, 500.0, 100.0, 500.0]).reshape(4, 2)

# Totally "wrong" aspect ratio symmetric trapezoid
src_pts = np.float32([170, 151, 470, 151, 610, 368, 30, 368]).reshape(4, 2)
H_mat = cv2.getPerspectiveTransform(src_pts, dst_pts)

# Draw a symmetric V-shape of parallel lanes in the image
# Let's say, 5 pixels outside the picked trapezoid, simulating the true lane markings
y_vals = np.linspace(151, 368, 50)
x_left = np.interp(y_vals, [151, 368], [170-5, 30-5])
x_right = np.interp(y_vals, [151, 368], [470+5, 610+5])

pts = np.vstack([x_left, y_vals, np.ones_like(y_vals)]).T
pts_bev = (H_mat @ pts.T).T
pts_bev = pts_bev[:, :2] / pts_bev[:, 2:]
print("Left lane transformed xs:", pts_bev[[0, 10, 25, 49], 0])

pts2 = np.vstack([x_right, y_vals, np.ones_like(y_vals)]).T
pts_bev2 = (H_mat @ pts2.T).T
pts_bev2 = pts_bev2[:, :2] / pts_bev2[:, 2:]
print("Right lane transformed xs:", pts_bev2[[0, 10, 25, 49], 0])


import cv2
import numpy as np

# Same setup as scratch12.py, but now the lanes are shifted
W = 640; H = 480
BEV_W = 400; BEV_H = 500
dst_pts = np.float32([100.0, 0.0, 300.0, 0.0, 300.0, 500.0, 100.0, 500.0]).reshape(4, 2)
src_pts = np.float32([170, 151, 470, 151, 610, 368, 30, 368]).reshape(4, 2)
H_mat = cv2.getPerspectiveTransform(src_pts, dst_pts)

# Let's say the car shifted right. So the lane lines in the image shift left!
# We can simulate this by shifting the input points.
shift = -20 # 20 pixels left in the image
y_vals = np.linspace(151, 368, 50)
x_left = np.interp(y_vals, [151, 368], [170, 30]) + shift
x_right = np.interp(y_vals, [151, 368], [470, 610]) + shift

ptsL = np.vstack([x_left, y_vals, np.ones_like(y_vals)]).T
pts_bevL = (H_mat @ ptsL.T).T
pts_bevL = pts_bevL[:, :2] / pts_bevL[:, 2:]

ptsR = np.vstack([x_right, y_vals, np.ones_like(y_vals)]).T
pts_bevR = (H_mat @ ptsR.T).T
pts_bevR = pts_bevR[:, :2] / pts_bevR[:, 2:]

center_xs = (pts_bevL[:, 0] + pts_bevR[:, 0]) / 2.0
print("Center line xs in BEV:")
print(center_xs[[0, 10, 25, 49]])

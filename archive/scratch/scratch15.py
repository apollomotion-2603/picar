import numpy as np
import cv2

# Camera params
pitch = 0.698131 # 40 degrees
fov = 1.3962634  # 80 degrees
W = 640; H = 480
f = (W / 2) / np.tan(fov / 2)

# Camera pose
# body_link to ground is 0.06m (wheel_radius=0.04 + axle offset = 0.02)
# camera is 0.2m above body_link.
# So camera is 0.26m above ground.
cam_height = 0.26

K = np.array([
    [f, 0, W/2],
    [0, f, H/2],
    [0, 0, 1]
], dtype=float)

# Rotation vector: pitch downward by 40 degrees.
# OpenCV coordinate system: Z is forward, Y is down, X is right.
# Our world: X is forward, Y is left, Z is up.
# Camera looking forward:
# World X -> Cam Z
# World Y -> Cam -X
# World Z -> Cam -Y
R_w2c_base = np.array([
    [0, -1, 0],
    [0, 0, -1],
    [1, 0, 0]
], dtype=float)

# Pitch 40 degrees down. Rotate around Cam X axis by 40 degrees.
# Wait, pitch down means pointing towards ground (positive Y in camera).
# So rotation around +X axis by +40 degrees.
theta = pitch
R_pitch = np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])
R_w2c = R_pitch @ R_w2c_base
rvec, _ = cv2.Rodrigues(R_w2c)

# Translation: camera is at world (X=0.14, Y=0, Z=0.26)
cam_pos_w = np.array([0.14, 0.0, 0.26])
# T_w2c = -R_w2c * cam_pos_w
tvec = -R_w2c @ cam_pos_w

# Let's project a perfect 0.5m wide rectangle from X_front to X_back
X_front = 0.4  # m, just ahead of bumper
X_back  = 1.4  # m, 1.0 meter long
Y_left  = 0.25 # m
Y_right = -0.25 # m

pts_3d = np.array([
    [X_back, Y_left, 0],   # Top-Left in BEV
    [X_back, Y_right, 0],  # Top-Right in BEV
    [X_front, Y_right, 0], # Bottom-Right in BEV
    [X_front, Y_left, 0]   # Bottom-Left in BEV
], dtype=float)

imgpts, _ = cv2.projectPoints(pts_3d, rvec, tvec, K, None)
imgpts = imgpts.reshape(-1, 2)

print("Projected exact rectangle:")
for p in imgpts:
    print(f"  {p[0]:.1f}, {p[1]:.1f}")
    
# Format for bev_src:
# Top-Left, Top-Right, Bottom-Right, Bottom-Left
print("bev_src: [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]".format(
    imgpts[0][0], imgpts[0][1],
    imgpts[1][0], imgpts[1][1],
    imgpts[2][0], imgpts[2][1],
    imgpts[3][0], imgpts[3][1],
))

# We mapped a 1.0 meter long by 0.5 meter wide rectangle.
# Let's say bev_dst has width 200 pixels for 0.5m -> 400px/m.
# Then 1.0m length should be exactly 400 BEV pixels.
# Left edge at X=100. Right edge at X=300.
# So dst points:
# Top-Left: (100, 500 - 400) = (100, 100)
# Top-Right: (300, 100)
# Bottom-Right: (300, 500)
# Bottom-Left: (100, 500)
print("bev_dst: [100.0, 100.0, 300.0, 100.0, 300.0, 500.0, 100.0, 500.0]")


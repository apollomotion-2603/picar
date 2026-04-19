import numpy as np
import cv2

W = 640; H = 480
pitch = 0.698131  # rad, from parameters.yaml
fov = 1.3962634   # rad
f = (W / 2) / np.tan(fov / 2)

cam_pos_z = 0.26 # m above ground

# World to Camera:
# Camera looks down by 40 degrees.
# World: X forward, Y left, Z up.
# Camera Base (0 pitch):
# Z_c = X_w
# X_c = -Y_w
# Y_c = -Z_w

# R_base
R_base = np.array([
    [ 0, -1,  0],
    [ 0,  0, -1],
    [ 1,  0,  0]
], dtype=float)

# Pitch downward means rotating around X_c by positive pitch
sp = np.sin(pitch)
cp = np.cos(pitch)
R_pitch = np.array([
    [1, 0, 0],
    [0, cp,-sp],
    [0, sp, cp]
])

R = R_pitch @ R_base

# Translation:
# Camera world position: [0.14, 0, 0.26]
# Actually, the camera position X offset doesn't matter for bev_src shape, it just shifts the ground.
# Let's assume camera is at [0, 0, cam_pos_z]
T_w = np.array([0, 0, cam_pos_z])
# V_c = R * (V_w - T_w) = R*V_w - R*T_w
T = -R @ T_w

def project(X_w, Y_w):
    V_w = np.array([X_w, Y_w, 0.0])
    V_c = R @ V_w + T
    if V_c[2] <= 0: return None
    u = f * V_c[0] / V_c[2] + W/2
    v = f * V_c[1] / V_c[2] + H/2
    return (u, v)

# Let's find rows on the ground that project to y=200 and y=368 in the image
# Actually, let's trace from X=0.3m to X=2.0m
for X in np.linspace(0.3, 1.5, 10):
    pL = project(X, 0.25)
    pR = project(X, -0.25)
    if pL:
        # print(f"X={X:.2f}m -> v={pL[1]:.1f}, uL={pL[0]:.1f}, uR={pR[0]:.1f}")
        pass

# Let's find X_bot that gives v = 400 (safely above the bumper at the bottom of the image)
# Let's find X_top that gives some length.
# If bev_dst is [100, 0, 300, 0, 300, 500, 100, 500]
# Width = 200px = 0.5m. So scale = 0.0025 m/px.
# Height = 500px = 1.25m.
# So we need a ground rectangle of width 0.5m and length 1.25m.
# Let's pick X_bot = 0.5m (close to car).
X_bot = 0.5
X_top = X_bot + 1.25 # 1.75m

p_tl = project(X_top, 0.25)
p_tr = project(X_top, -0.25)
p_br = project(X_bot, -0.25)
p_bl = project(X_bot, 0.25)

print(f"bev_src: [{p_tl[0]:.1f}, {p_tl[1]:.1f}, {p_tr[0]:.1f}, {p_tr[1]:.1f}, {p_br[0]:.1f}, {p_br[1]:.1f}, {p_bl[0]:.1f}, {p_bl[1]:.1f}]")

import numpy as np

# Camera params
pitch = 0.698131 # 40 degrees
fov = 1.3962634  # 80 degrees
W = 640
H = 480
f = (W / 2) / np.tan(fov / 2)

# Camera mounts
# body_length: 0.3
# camera_stick_joint is at x=0.3/2 - 0.02/2 = 0.14. z=0.2/2=0.1. relative to body_link
# body_link is at z=wheel_radius=0.04 (approx). Actually wheelbase z.
# Let's say camera height from ground is exactly H_c
# The base footprint is at z=0. The body origin is probably z ~ 0.05.
H_c = 0.25 # guess

# Rotation from Camera to World
# Camera looks down: pitch > 0.
# In world: x is forward, y is left, z is up.
# Camera: Z is forward optical ray, X is right, Y is down.
# Base Camera (no pitch):
# X_c = -Y_w
# Y_c = -Z_w
# Z_c = X_w
R_base = np.array([
    [ 0, -1,  0],
    [ 0,  0, -1],
    [ 1,  0,  0]
])

# Pitch downward by 40 deg around X_c axis
c_p = np.cos(pitch)
s_p = np.sin(pitch)
R_pitch = np.array([
    [1,    0,     0],
    [0,  c_p,  -s_p],
    [0,  s_p,   c_p]
])

R_c2w = R_base @ R_pitch.T  # wait, R_pitch is applied in camera coords

def proj(X_c, Y_c, Z_c):
    # Pinhole projection
    u = f * X_c / Z_c + W / 2
    v = f * Y_c / Z_c + H / 2
    return u, v

def world2img(X_w, Y_w, Z_w=-0.25):
    # Vector from camera to point
    # V_w = [X_w, Y_w, Z_w]
    V_w = np.array([X_w, Y_w, Z_w])
    # V_c = R_c2w^-1 * V_w = R_c2w.T * V_w
    # Wait, let's just make it simple:
    # Forward depth X_w
    # Point in camera frame:
    # x_c = -Y_w (right)
    # y_c = Z_w is negative. 
    # Let's apply standard Rx(pitch)
    # X_w = Z_c * cos(pitch) + Y_c * sin(pitch)
    # Z_w = -Z_c * sin(pitch) + Y_c * cos(pitch)
    # Y_w = -X_c
    
    Y_w_p = -Y_w
    # solve for Y_c, Z_c
    # -Z_w = Z_c*s - Y_c*c
    # X_w = Z_c*c + Y_c*s
    # Z_c*c + Y_c*s = X
    # Z_c*s - Y_c*c = -Z
    
    Z_c = X_w * c_p - Z_w * s_p
    Y_c = X_w * s_p + Z_w * c_p
    X_c = Y_w_p
    
    return proj(X_c, Y_c, Z_c)

# We want lane lines: Y_w = +0.25 and Y_w = -0.25 (0.5m apart)
# Let's see pixels for X_w = 0.5 (close) and X_w = 1.5 (far)
print("Close left lane (0.5, 0.25):", world2img(0.5, 0.25))
print("Close right lane (0.5, -0.25):", world2img(0.5, -0.25))
print("Far left lane (1.5, 0.25):", world2img(1.5, 0.25))
print("Far right lane (1.5, -0.25):", world2img(1.5, -0.25))

# Let's try to match the user's calibration: Y is roughly 151 at top, 368 at bottom
# Find X_w that gives v=368
for X in np.linspace(0.1, 2.0, 100):
    u, v = world2img(X, 0.25)
    if abs(v - 368) < 5:
        print(f"X_w={X:.2f} -> v={v:.1f}. u={u:.1f}")

for X in np.linspace(0.1, 3.0, 100):
    u, v = world2img(X, 0.25)
    if abs(v - 151) < 5:
        print(f"X_w={X:.2f} -> v={v:.1f}. u={u:.1f}")


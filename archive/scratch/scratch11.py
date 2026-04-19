import numpy as np

# Camera params
pitch = 0.698131 # 40 degrees
fov = 1.3962634  # 80 degrees
W = 640
H = 480
f = (W / 2) / np.tan(fov / 2)
c_p = np.cos(pitch)
s_p = np.sin(pitch)

def world2img(X_w, Y_w, Z_w=-0.26):
    Y_w_p = -Y_w
    Z_c = X_w * c_p - Z_w * s_p
    Y_c = X_w * s_p + Z_w * c_p
    X_c = Y_w_p
    
    u = f * X_c / Z_c + W / 2
    v = f * Y_c / Z_c + H / 2
    return u, v

# Let's find the v coordinate for X=0.3m (close) and X=1.5m (far)
print("Close (X=0.5m)")
u_cl, v_c = world2img(0.25, 0.5)
u_cr, _   = world2img(-0.25, 0.5)
print(f"  v={v_c:.1f}, u_l={u_cl:.1f}, u_r={u_cr:.1f}, width={u_cr-u_cl:.1f}")

print("Far (X=2.0m)")
u_fl, v_f = world2img(0.25, 2.0)
u_fr, _   = world2img(-0.25, 2.0)
print(f"  v={v_f:.1f}, u_l={u_fl:.1f}, u_r={u_fr:.1f}, width={u_fr-u_fl:.1f}")


# Let's pick a nice source quadrilateral.
# Let's choose the bottom of BEV to be exactly v=400 in the image, and top to be v=150.
def get_x_for_v(v_target):
    # u, v = world2img(X, Y)
    # v = f * (X*s_p + Z*c_p) / (X*c_p - Z*s_p) + H/2
    # we need to find X given v.
    for X in np.linspace(0.1, 5.0, 10000):
        _, v = world2img(0, X)
        if abs(v - v_target) < 0.5:
            return X
    return None

X_bot = get_x_for_v(400)
X_top = get_x_for_v(150)

print(f"Target v=400 corresponds to X_w={X_bot:.3f}")
print(f"Target v=150 corresponds to X_w={X_top:.3f}")

# Now, we need the BEV to be 300 pixels wide (from 50 to 350 for example).
# But what physical width does that represent in X_w?
# Lane is ~0.5m wide.
u_tl, _ = world2img(0.25, X_top)
u_tr, _ = world2img(-0.25, X_top)
u_bl, _ = world2img(0.25, X_bot)
u_br, _ = world2img(-0.25, X_bot)

print(f"SRC Quad for width=0.5m at v=150 and v=400:")
print(f"[{u_tl:.1f}, 150.0, {u_tr:.1f}, 150.0, {u_br:.1f}, 400.0, {u_bl:.1f}, 400.0]")

# Wait, if we use the same BEV scale: width=0.5m -> 200px.
# We map this to bev_dst: [100.0, 0.0, 300.0, 0.0, 300.0, 500.0, 100.0, 500.0]
# Does the physical length (X_top - X_bot) map to 500 BEV pixels correctly?
length_w = X_top - X_bot
print(f"Physical length = {length_w:.3f} m")
print(f"If scale is 0.0025 m/px, 500 pixels is {500 * 0.0025} m.")
print(f"We need the physical length to be exactly {0.0025 * 500} m = 1.25 m.")

# Find X_top such that X_top - X_bot = 1.25
X_top_needed = X_bot + 1.25
_, v_top_needed = world2img(0, X_top_needed)

print(f"To get exactly 1.25m length, we must use X_top = {X_top_needed:.3f}, which gives v={v_top_needed:.1f}")

u_tl2, _ = world2img(0.25, X_top_needed)
u_tr2, _ = world2img(-0.25, X_top_needed)

print(f"NEW SRC Quad (perfect metric mapping to dst!):")
print(f"[{u_tl2:.1f}, {v_top_needed:.1f}, {u_tr2:.1f}, {v_top_needed:.1f}, {u_br:.1f}, 400.0, {u_bl:.1f}, 400.0]")



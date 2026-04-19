import numpy as np
import cv2

# True camera specs:
W = 640; H = 480
pitch = 0.698131  # 40 deg
fov = 1.3962634

# We found from intersection of lines that symmetric vanishing point is Y=-81.5
# If looking down, the vanishing line is ABOVE the center.
# Let's trust the vanishing point Y=-81.5 from the image directly!
v_horizon = -81.5

# Let's pick a BEV rectangle of width 200px (0.5m) and height 500px (1.25m)
# Let's use the bottom boundary as v_bot = 400 (to avoid bumper).
# Let's find v_top.
# Distance on ground Z is proportional to 1/(v - v_horizon).
# Z_bot ~ 1 / (400 - (-81.5)) = 1 / 481.5
# We need Z_top - Z_bot to have a specific ratio to the width.
# As established before, if Width = 0.5m and Length = 1.25m, Length/Width = 2.5
# On the image, the width of the lane at any row v is W_img(v).
# From the symmetric points:
# At v=151, width = 470 - 170 = 300 pixels
# At v=368, width = 610 - 30 = 580 pixels
# So W_img(v) = m * (v - v_horizon).
# Let's check: 300 / (151 - (-81.5)) = 300 / 232.5 = 1.290
# 580 / (368 - (-81.5)) = 580 / 449.5 = 1.290
# The constant is exactly 1.2903!
m_w = 1.29032258

# W_img(v) is the pixel width in the image.
# Physical width is 0.5m.
# Physical depth Z(v) = (f_y * 0.5) / W_img(v).
# Since W_img(v) = m_w * (v - v_horizon),
# Z(v) = (f_y * 0.5) / (m_w * (v - v_horizon))
# We don't even need f_y! Because we want Z(v_top) - Z(v_bot) = 1.25.
# Z(v_top) - Z(v_bot) = (f_y * 0.5 / m_w) * [ 1/(v_top - v_horizon) - 1/(v_bot - v_horizon) ]
# So 1.25 = (f_y * 0.5 / m_w) * [ 1/(v_top - (-81.5)) - 1/(400 - (-81.5)) ]

# Wait, how do we find f_y?
# f_y = H/2 / tan(fov/2) approx.
# In Gazebo, horizontal fov is 1.3962634
f_x = (640 / 2) / np.tan(1.3962634 / 2) # = 382.7
# Assuming square pixels, f_y = f_x = 382.7

Z_constant = f_x * 0.5 / m_w # = 382.7 * 0.5 / 1.2903 = 148.3

# We want length = 1.25m:
# 1.25 = 148.3 * [ 1/(v_top + 81.5) - 1 / 481.5 ]
# 1.25 / 148.3 = 1/(v_top + 81.5) - 0.0020768
# 0.008428 + 0.0020768 = 1/(v_top + 81.5)
# 0.010505 = 1/(v_top + 81.5)
# v_top + 81.5 = 95.19
# v_top = 13.69 !

# Let's verify! If v_bot = 400 and v_top = 13.7, the area is huge!
# Is it valid? Yes, it gives EXACTLY a 1.25m long by 0.5m wide rectangle!
# Let's compute the four corners:
# At v_bot = 400, W_img = 1.2903 * (400 + 81.5) = 621.3
# Center is x = 320.
# Left_bot = 320 - 621.3/2 = 9.35
# Right_bot = 320 + 621.3/2 = 630.65

# At v_top = 13.7, W_img = 1.2903 * (13.7 + 81.5) = 122.8
# Left_top = 320 - 122.8/2 = 258.6
# Right_top = 320 + 122.8/2 = 381.4

print(f"Top: {258.6:.1f}, 13.7  and  {381.4:.1f}, 13.7")
print(f"Bot: {9.4:.1f}, 400.0  and  {630.7:.1f}, 400.0")


import numpy as np

p1 = np.array([172.0, 151.0])
p2 = np.array([29.0, 368.0])
v1 = p1 - p2

p3 = np.array([471.0, 151.0])
p4 = np.array([609.0, 368.0])
v2 = p3 - p4

# line 1: p1 + t1 * v1
# line 2: p3 + t2 * v2
# p1x + t1*v1x = p3x + t2*v2x
# p1y + t1*v1y = p3y + t2*v2y

A = np.array([[v1[0], -v2[0]], [v1[1], -v2[1]]])
b = p3 - p1
t = np.linalg.solve(A, b)

vp = p1 + t[0] * v1
print("Vanishing point:", vp)

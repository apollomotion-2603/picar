import numpy as np
p1 = np.array([170.0, 151.0])
p2 = np.array([30.0, 368.0])
v1 = p1 - p2

p3 = np.array([470.0, 151.0])
p4 = np.array([610.0, 368.0])
v2 = p3 - p4

A = np.array([[v1[0], -v2[0]], [v1[1], -v2[1]]])
b = p3 - p1
t = np.linalg.solve(A, b)
vp = p1 + t[0] * v1
print("Symmetric VP:", vp)

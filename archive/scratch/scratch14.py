import numpy as np

# Let's say pc[1] = -0.001.
SCALE = 0.0025
print("kappa formula in visualizer:", 2.0 * (-0.001) * SCALE**2)
# Wait, this gives -1.25e-08.
print("kappa formula true:", 2.0 * (-0.001) / SCALE)
# This gives -0.8

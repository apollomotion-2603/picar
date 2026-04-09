import sys
import os
import rclpy
from rclpy.node import Node
sys.path.append(os.path.expanduser('~/main/1_project/1_autonomous_driving/ros2_ws/install/mpc_pkg/lib/python3.12/site-packages'))
from mpc_pkg.mpc_node import NMPCNode
import numpy as np

rclpy.init()
node = NMPCNode()

# Force x0
node.x_est = np.array([0.0, 0.002, 0.001, 0.11])  # slightly off center, slow velocity
node.kappa = 0.030
node.v_ref_adaptive = 1.21

node._warm_start(node.x_est)
node.solver.set(0, 'lbx', node.x_est)
node.solver.set(0, 'ubx', node.x_est)

for i in range(node.N + 1):
    node.solver.set(i, 'p', np.array([node.kappa]))

v_ub = np.array([node.DELTA_MAX, node.v_ref_adaptive])
for i in range(node.N):
    node.solver.constraints_set(i, 'ubu', v_ub)
    node.solver.constraints_set(i, 'lbu', np.array([-node.DELTA_MAX, node.V_MIN]))

status = node.solver.solve()
u_opt = node.solver.get(0, 'u')
print("Status:", status)
print("Optimal Command (delta, v_cmd):", u_opt)
print("x_opt at step 1:", node.solver.get(1, 'x'))

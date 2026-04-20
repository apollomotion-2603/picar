#!/usr/bin/env python3
"""Analyze bag_map_test_7 — extract n, alpha, kappa, v_cmd, delta_cmd, tf pose."""
import sys
import numpy as np
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else str(
    Path(__file__).parent / 'bag_map_test_7')

reader = SequentialReader()
reader.open(
    StorageOptions(uri=BAG, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr',
                     output_serialization_format='cdr'))

type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

data = {k: [] for k in [
    'lane', 'ekf', 'delta', 'vel', 'enabled', 'tf']}

while reader.has_next():
    topic, raw, ts = reader.read_next()
    t = ts * 1e-9
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(raw, msg_type)

    if topic == '/perception/lane_state':
        data['lane'].append((t, msg.e_y, msg.e_psi, msg.kappa,
                             msg.coeff_a, msg.coeff_b, msg.coeff_c, msg.coeff_d,
                             msg.s_max, msg.lane_detected))
    elif topic == '/ekf/vehicle_state':
        data['ekf'].append((t, msg.n, msg.alpha, msg.v, msg.ekf_healthy))
    elif topic == '/steering_angle':
        data['delta'].append((t, msg.data))
    elif topic == '/velocity':
        data['vel'].append((t, msg.data))
    elif topic == '/car_enabled':
        data['enabled'].append((t, msg.data))
    elif topic == '/tf':
        for tfm in msg.transforms:
            if tfm.child_frame_id in ('ackermann_steering_vehicle/base_link',
                                       'base_link', 'ackermann_steering_vehicle'):
                tr = tfm.transform.translation
                rot = tfm.transform.rotation
                data['tf'].append((t, tr.x, tr.y, tr.z,
                                   rot.x, rot.y, rot.z, rot.w))

# Normalize time to start
t0 = data['ekf'][0][0] if data['ekf'] else 0
for k, arr in data.items():
    data[k] = np.array([(row[0] - t0, *row[1:]) for row in arr], dtype=object)

print(f"=== Summary ===")
print(f"Duration: {data['ekf'][-1][0]:.2f}s")
print(f"Lane msgs: {len(data['lane'])}, EKF: {len(data['ekf'])}, "
      f"delta: {len(data['delta'])}, vel: {len(data['vel'])}, tf: {len(data['tf'])}")

# Identify start of motion (car_enabled True)
enabled = data['enabled']
t_start = None
for row in enabled:
    if row[1]:
        t_start = row[0]
        break
print(f"car_enabled=True at t={t_start:.2f}s" if t_start is not None
      else "Never enabled")

# Stats per section: divide test into thirds
T = data['ekf'][-1][0]
sections = [('early', 0, T/3), ('middle', T/3, 2*T/3), ('late', 2*T/3, T)]

print("\n=== EKF stats (n_mm, alpha_deg, v_mps) per section ===")
for name, ta, tb in sections:
    rows = [r for r in data['ekf'] if ta <= r[0] <= tb and r[4]]  # healthy
    if not rows:
        continue
    n = np.array([r[1] for r in rows]) * 1000
    a = np.degrees([r[2] for r in rows])
    v = np.array([r[3] for r in rows])
    print(f" {name:6s} [{ta:5.1f}-{tb:5.1f}s] "
          f"n={n.mean():+6.1f}±{n.std():.1f}mm "
          f"α={a.mean():+5.1f}±{a.std():.1f}° "
          f"v={v.mean():.2f}±{v.std():.2f}m/s "
          f"|n|max={np.abs(n).max():.1f}mm")

print("\n=== Perception kappa stats per section ===")
for name, ta, tb in sections:
    rows = [r for r in data['lane'] if ta <= r[0] <= tb and r[9]]  # detected
    if not rows:
        continue
    k = np.array([r[3] for r in rows])
    ey = np.array([r[1] for r in rows]) * 1000
    print(f" {name:6s} κ={k.mean():+.4f}±{k.std():.4f} "
          f"|κ|max={np.abs(k).max():.4f} "
          f"ey={ey.mean():+.1f}±{ey.std():.1f}mm")

print("\n=== Control cmd stats per section ===")
for name, ta, tb in sections:
    drows = [r for r in data['delta'] if ta <= r[0] <= tb]
    vrows = [r for r in data['vel'] if ta <= r[0] <= tb]
    if not drows: continue
    d = np.degrees([r[1] for r in drows])
    v = np.array([r[1] for r in vrows])
    print(f" {name:6s} δ={d.mean():+5.1f}±{d.std():.1f}° δmax={np.abs(d).max():.1f}° "
          f"v_cmd={v.mean():.2f}±{v.std():.2f} vmin={v.min():.2f}")

# Detect oscillation: delta sign flips per second in last section
print("\n=== Delta sign flips (oscillation) per section ===")
for name, ta, tb in sections:
    drows = [r for r in data['delta'] if ta <= r[0] <= tb]
    if len(drows) < 2: continue
    d = np.array([r[1] for r in drows])
    signs = np.sign(d)
    flips = np.sum(np.diff(signs) != 0)
    duration = tb - ta
    print(f" {name:6s} flips={flips} rate={flips/duration:.1f}/s")

# Trajectory (x, y from tf)
if len(data['tf']) > 0:
    xs = np.array([r[1] for r in data['tf']])
    ys = np.array([r[2] for r in data['tf']])
    print(f"\n=== TF trajectory ===")
    print(f" x: [{xs.min():.2f}, {xs.max():.2f}]m (range {xs.max()-xs.min():.2f}m)")
    print(f" y: [{ys.min():.2f}, {ys.max():.2f}]m (range {ys.max()-ys.min():.2f}m)")

# Late-section deep dive: last 10 seconds of motion
print("\n=== Late section deep-dive (last 10s) ===")
t_late = T - 10
lane_late = [r for r in data['lane'] if r[0] >= t_late and r[9]]
if lane_late:
    for r in lane_late[::max(1, len(lane_late)//15)]:
        t, ey, epsi, k, a, b, c, d, smax, det = r
        # Find closest delta cmd
        dcmd = min(data['delta'], key=lambda x: abs(x[0]-t))
        vcmd = min(data['vel'], key=lambda x: abs(x[0]-t))
        print(f" t={t:5.2f}s ey={ey*1000:+6.1f}mm ε={np.degrees(epsi):+5.1f}° "
              f"κ={k:+.4f} δ={np.degrees(dcmd[1]):+5.1f}° v={vcmd[1]:.2f}")

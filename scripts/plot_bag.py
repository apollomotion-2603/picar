#!/usr/bin/env python3
"""
Plot steering angle + perception data từ rosbag.
Usage:
  python3 scripts/plot_bag.py bag_map1_test
  python3 scripts/plot_bag.py bag_map1_test --topics steering kappa
"""
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64
from rosidl_runtime_py.utilities import get_message

def read_bag(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))

    topic_types = {}
    for info in reader.get_all_topics_and_types():
        topic_types[info.name] = info.type

    data = {}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(raw, msg_type)
        if topic not in data:
            data[topic] = {'t': [], 'msg': []}
        data[topic]['t'].append(t_ns * 1e-9)
        data[topic]['msg'].append(msg)

    # Normalize time to start at 0
    t_min = min(d['t'][0] for d in data.values() if d['t'])
    for d in data.values():
        d['t'] = [t - t_min for t in d['t']]

    return data


def plot_steering(data):
    if '/steering_angle' not in data:
        print("No /steering_angle in bag")
        return
    d = data['/steering_angle']
    t = d['t']
    vals = [np.degrees(m.data) for m in d['msg']]

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, vals, linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Steering Angle (deg)')
    ax.set_title('Steering Angle over Time')
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_kappa(data):
    if '/perception/lane_state' not in data:
        print("No /perception/lane_state in bag")
        return
    d = data['/perception/lane_state']
    t = [d['t'][i] for i, m in enumerate(d['msg']) if m.lane_detected]
    kappa = [m.kappa for m in d['msg'] if m.lane_detected]

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, kappa, linewidth=0.8, color='tab:orange')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Kappa (1/m)')
    ax.set_title('Perception Kappa over Time')
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_lane_state(data):
    if '/perception/lane_state' not in data:
        print("No /perception/lane_state in bag")
        return
    d = data['/perception/lane_state']
    detected = [(d['t'][i], m) for i, m in enumerate(d['msg']) if m.lane_detected]
    if not detected:
        return
    t     = [x[0] for x in detected]
    e_y   = [x[1].e_y * 1000 for x in detected]  # mm
    e_psi = [np.degrees(x[1].e_psi) for x in detected]
    kappa = [x[1].kappa for x in detected]

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(t, e_y, linewidth=0.8)
    axes[0].set_ylabel('e_y (mm)')
    axes[0].set_title('Lane State from Perception')
    axes[0].axhline(0, color='gray', linestyle='--', linewidth=0.5)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, e_psi, linewidth=0.8, color='tab:green')
    axes[1].set_ylabel('e_psi (deg)')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, kappa, linewidth=0.8, color='tab:orange')
    axes[2].set_ylabel('kappa (1/m)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_all(data):
    """Steering + e_y + kappa trên cùng 1 figure."""
    has_steer = '/steering_angle' in data
    has_lane  = '/perception/lane_state' in data
    if not has_steer and not has_lane:
        print("No relevant topics found")
        return

    n_plots = (1 if has_steer else 0) + (3 if has_lane else 0)
    fig, axes = plt.subplots(n_plots, 1, figsize=(14, 3 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]
    idx = 0

    if has_steer:
        d = data['/steering_angle']
        axes[idx].plot(d['t'], [np.degrees(m.data) for m in d['msg']],
                       linewidth=0.8)
        axes[idx].set_ylabel('Steering (deg)')
        axes[idx].set_title('Steering + Perception Analysis')
        axes[idx].axhline(0, color='gray', linestyle='--', linewidth=0.5)
        axes[idx].grid(True, alpha=0.3)
        idx += 1

    if has_lane:
        d = data['/perception/lane_state']
        det = [(d['t'][i], m) for i, m in enumerate(d['msg']) if m.lane_detected]
        t     = [x[0] for x in det]
        e_y   = [x[1].e_y * 1000 for x in det]
        e_psi = [np.degrees(x[1].e_psi) for x in det]
        kappa = [x[1].kappa for x in det]

        axes[idx].plot(t, e_y, linewidth=0.8, color='tab:blue')
        axes[idx].set_ylabel('e_y (mm)')
        axes[idx].grid(True, alpha=0.3)
        idx += 1

        axes[idx].plot(t, e_psi, linewidth=0.8, color='tab:green')
        axes[idx].set_ylabel('e_psi (deg)')
        axes[idx].grid(True, alpha=0.3)
        idx += 1

        axes[idx].plot(t, kappa, linewidth=0.8, color='tab:orange')
        axes[idx].set_ylabel('kappa (1/m)')
        axes[idx].grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.tight_layout()
    return fig


PLOT_MAP = {
    'steering': plot_steering,
    'kappa': plot_kappa,
    'lane': plot_lane_state,
    'all': plot_all,
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot rosbag data')
    parser.add_argument('bag_path', help='Path to rosbag directory')
    parser.add_argument('--topics', nargs='+', default=['all'],
                        choices=list(PLOT_MAP.keys()),
                        help='What to plot (default: all)')
    args = parser.parse_args()

    print(f'Reading {args.bag_path} ...')
    data = read_bag(args.bag_path)
    print(f'Topics: {list(data.keys())}')
    print(f'Duration: {max(d["t"][-1] for d in data.values() if d["t"]):.1f}s')

    for name in args.topics:
        fig = PLOT_MAP[name](data)
        if fig:
            fig.savefig(f'{args.bag_path}_{name}.png', dpi=150)
            print(f'Saved: {args.bag_path}_{name}.png')

    plt.show()

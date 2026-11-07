#!/usr/bin/env python3
"""obstacle_avoidance düğümünü izler (Nav2 olmadan).

Belirtilen süre boyunca:
  - clear   : gövdenin (footprint) engele en yakın mesafesi [m]
  - path    : gerçek konuma (/ground_truth) göre katedilen yol [m]
  - stuck   : 10 s'lik pencerelerde 0.1 m'den az ilerlenen pencere sayısı
Çarpma (clear < 0.02 m) veya hiç ilerleme yoksa çıkış kodu 1.

  python3 tools/avoidance_monitor.py --duration 90
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from nav_goal_test import LASER_X, footprint_clearance


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=90.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node('avoidance_monitor')
    state = {'clear': float('inf'), 'pos': None, 'path': 0.0}

    def on_scan(msg):
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                state['clear'] = min(
                    state['clear'], footprint_clearance(LASER_X + r * math.cos(a), r * math.sin(a)))

    def on_truth(msg):
        p = msg.pose.pose.position
        if state['pos'] is not None:
            state['path'] += math.hypot(p.x - state['pos'][0], p.y - state['pos'][1])
        state['pos'] = (p.x, p.y)

    node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, 'ground_truth', on_truth, 10)

    # Robot hareket etmeye başlayana kadar bekle (Gazebo + düğüm açılışı).
    t_wait = time.monotonic()
    while state['pos'] is None and time.monotonic() - t_wait < 60.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    t0 = time.monotonic()
    window_start, window_path, stuck = t0, 0.0, 0
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - window_start > 10.0:
            if state['path'] - window_path < 0.1:
                stuck += 1
            window_start, window_path = time.monotonic(), state['path']

    ok = state['clear'] > 0.02 and state['path'] > 1.0
    print(f'{"OK" if ok else "FAIL"}: {args.duration:.0f}s, path={state["path"]:.1f}m, '
          f'clear={state["clear"]:.2f}m, stuck_windows={stuck}', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

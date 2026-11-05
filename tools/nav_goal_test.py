#!/usr/bin/env python3
"""Nav2 hedef testi.

Simülasyon + Nav2 çalışırken sırayla hedefler gönderir ve her hedef için şunları yazar:
  - sonuç, süre, kurtarma (recovery) sayısı
  - clear : robot GÖVDESİNİN (footprint dikdörtgeni) engele en yakın mesafesi [m]
  - goal  : hedef ile Nav2'nin bildiği konum (map -> base_footprint) farkı [m, rad]
  - loc   : AMCL tahmini ile Gazebo gerçek konumu (/ground_truth) farkı [m]
En sonda özet basar; herhangi bir hedef başarısız olursa çıkış kodu 1 olur.

Kullanım:
  python3 tools/nav_goal_test.py              # varsayılan hedef listesi
  python3 tools/nav_goal_test.py --rounds 3   # listeyi 3 kez tekrarla
  python3 tools/nav_goal_test.py --timeout 90
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# base_footprint çerçevesinde gövde dikdörtgeni (nav2_params.yaml footprint ile aynı)
FOOT_X = (-0.10, 0.40)
FOOT_Y = (-0.19, 0.19)
# laser_frame'in base_footprint'e göre konumu (URDF: 0.15 + 0.10)
LASER_X = 0.25

# (isim, x, y, yaw) - map çerçevesinde. test_world.world için seçildi.
GOALS = [
    ('behind_divider', 2.0, -2.0, -0.785),
    ('north_east', 2.2, 2.2, 1.57),
    ('north_west', -2.2, 2.2, 3.14),
    ('south_west', -2.2, -2.3, -1.57),
    ('east_mid', 2.3, 0.0, 0.0),
    ('divider_gap', 0.4, -2.3, 0.0),
    # Zor hedefler: inflation (renkli) bölgesinde, engele ~0.3 m.
    ('divider_back_close', 1.45, -1.55, 0.785),
    ('near_pillar', 1.5, 0.55, 3.14),
    ('home', 0.0, 0.0, 0.0),
]


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def footprint_clearance(px, py):
    """Noktanın gövde dikdörtgenine uzaklığı; içerideyse 0."""
    dx = max(FOOT_X[0] - px, 0.0, px - FOOT_X[1])
    dy = max(FOOT_Y[0] - py, 0.0, py - FOOT_Y[1])
    return math.hypot(dx, dy)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator()

    state = {'clear': float('inf'), 'truth': None, 'loc_err_max': 0.0}
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    def on_scan(msg):
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            c = footprint_clearance(LASER_X + r * math.cos(a), r * math.sin(a))
            if c < state['clear']:
                state['clear'] = c

    def on_truth(msg):
        state['truth'] = msg.pose.pose

    def amcl_pose():
        try:
            t = tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    nav.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    nav.create_subscription(Odometry, 'ground_truth', on_truth, 10)

    nav.waitUntilNav2Active()

    results = []
    for rnd in range(args.rounds):
        for name, x, y, yaw in GOALS:
            state['clear'] = float('inf')
            state['loc_err_max'] = 0.0
            t0 = time.monotonic()
            nav.goToPose(make_pose(nav, x, y, yaw))
            recoveries = 0
            timed_out = False
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    recoveries = fb.number_of_recoveries
                if time.monotonic() - t0 > args.timeout:
                    timed_out = True
                    nav.cancelTask()
                    # İptalin bitmesini bekle; yoksa sonraki hedef eski sonucu okur.
                    while not nav.isTaskComplete():
                        rclpy.spin_once(nav, timeout_sec=0.1)
                    break
                rclpy.spin_once(nav, timeout_sec=0.1)
                est = amcl_pose()
                if est is not None and state['truth'] is not None:
                    tp = state['truth'].position
                    state['loc_err_max'] = max(
                        state['loc_err_max'], math.hypot(est[0] - tp.x, est[1] - tp.y))
            dt = time.monotonic() - t0
            res = nav.getResult()

            err_xy = err_yaw = float('nan')
            est = amcl_pose()
            if est is not None:
                err_xy = math.hypot(est[0] - x, est[1] - y)
                d = est[2] - yaw
                err_yaw = abs(math.atan2(math.sin(d), math.cos(d)))

            ok = res == TaskResult.SUCCEEDED and not timed_out
            results.append((rnd, name, ok, dt, recoveries, state['clear'], err_xy, err_yaw,
                            state['loc_err_max']))
            print(f'[{rnd}] {name:18s} {"OK  " if ok else "FAIL"} {dt:6.1f}s '
                  f'rec={recoveries} clear={state["clear"]:.2f} '
                  f'goal={err_xy:.2f}m/{err_yaw:.2f}rad loc_max={state["loc_err_max"]:.2f}',
                  flush=True)

    print('\n=== ÖZET ===')
    n_ok = sum(1 for r in results if r[2])
    print(f'{n_ok}/{len(results)} hedef başarılı, '
          f'toplam kurtarma: {sum(r[4] for r in results)}, '
          f'ortalama süre: {sum(r[3] for r in results) / len(results):.1f}s, '
          f'en küçük gövde mesafesi: {min(r[5] for r in results):.2f}m, '
          f'en büyük lokalizasyon hatası: {max(r[8] for r in results):.2f}m')

    nav.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == '__main__':
    main()

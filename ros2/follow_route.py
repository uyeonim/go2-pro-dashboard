#!/usr/bin/env python3
"""저장한 경로(routes/<이름>.yaml)를 Nav2로 차례대로 따라가기 + 실제 이동 경로를 CSV로 기록.

  bash route.sh follow lab_loop            # 저장한 지도 모드(run_nav.sh)에서
  bash route.sh follow lab_loop --live     # 실시간 지도 모드(run_nav_live.sh)에서
  bash route.sh follow lab_loop --loop     # 끝나면 처음부터 반복 (Ctrl+C로 멈춤)

기록: routes/logs/<이름>_<시각>.csv  (t, x, y, yaw, 목표 번호) → 계획 경로와 비교용
Ctrl+C를 누르면 이동을 취소하고, 0.5초 안에 로봇이 멈춰요 (twist_mux 시간 제한).
"""
import argparse
import csv
import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTES = os.path.join(BASE, "routes")


def make_pose(nav, frame, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class PoseLogger:
    """map → base_link 위치를 따로 돌려서 0.2초마다 기록."""

    def __init__(self, frame):
        self.frame = frame
        self.node = rclpy.create_node("route_pose_logger")
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self.node)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.rows, self.target = [], 0
        self.t0 = time.time()
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def pose(self):
        try:
            t = self.buf.lookup_transform(self.frame, "base_link", rclpy.time.Time(), Duration(seconds=0.2))
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _run(self):
        while not self.stop.is_set():
            p = self.pose()
            if p:
                self.rows.append([round(time.time() - self.t0, 2), round(p[0], 3), round(p[1], 3),
                                  round(math.degrees(p[2]), 1), self.target])
            time.sleep(0.2)

    def save(self, name):
        self.stop.set()
        os.makedirs(os.path.join(ROUTES, "logs"), exist_ok=True)
        path = os.path.join(ROUTES, "logs", f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x", "y", "yaw_deg", "target_index"])
            w.writerows(self.rows)
        dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(self.rows, self.rows[1:]))
        return path, dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="routes/<이름>.yaml")
    ap.add_argument("--loop", action="store_true", help="끝나면 처음부터 반복")
    ap.add_argument("--live", action="store_true", help="실시간 지도 모드(run_nav_live.sh)일 때")
    args = ap.parse_args()

    path = os.path.join(ROUTES, f"{args.name}.yaml")
    if not os.path.exists(path):
        raise SystemExit(f"❌ 경로 파일이 없어요: {path}  (먼저: route.sh record {args.name})")
    with open(path, encoding="utf-8") as f:
        route = yaml.safe_load(f)
    frame, wps = route.get("frame", "map"), route["waypoints"]
    print(f"🧭 경로 '{args.name}': 점 {len(wps)}개 ({frame} 좌표)")

    rclpy.init()
    nav = BasicNavigator()
    # 실시간 지도 모드에는 AMCL이 없어서 위치 찾기 노드는 기다리지 않음
    nav.waitUntilNav2Active(localizer="robot_localization" if args.live else "amcl")
    logger = PoseLogger(frame)

    lap = 0
    try:
        while True:
            lap += 1
            poses = [make_pose(nav, frame, *wp) for wp in wps]
            print(f"\n▶ {lap}바퀴째 출발")
            t_start = time.time()
            nav.followWaypoints(poses)
            last = -1
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb and fb.current_waypoint != last:
                    last = logger.target = fb.current_waypoint
                    x, y, _ = wps[last]
                    print(f"  → {last + 1}/{len(wps)}번 점 ({x:.2f}, {y:.2f})로 이동 중")
                time.sleep(0.3)
            res = nav.getResult()
            took = time.time() - t_start
            if res == TaskResult.SUCCEEDED:
                print(f"✅ {lap}바퀴 완료 ({took:.0f}초)")
            elif res == TaskResult.CANCELED:
                print("⏹ 취소됨"); break
            else:
                print(f"❌ 실패 ({took:.0f}초) — 막힌 곳이 있거나 위치를 잃었을 수 있어요"); break
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\n⏹ Ctrl+C — 이동 취소")
        nav.cancelTask()
    finally:
        log_path, dist = logger.save(args.name)
        print(f"📝 실제 이동 기록: {log_path}  (이동 거리 {dist:.2f} m)")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

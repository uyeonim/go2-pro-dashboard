#!/usr/bin/env python3
"""RViz에서 "Publish Point"로 찍은 점들을 경로 파일(routes/<이름>.yaml)로 저장.

  bash route.sh record lab_loop
  → RViz 위쪽 "Publish Point" 버튼을 누르고 지도 위를 클릭 (점마다 다시 눌러야 함)
  → 다 찍었으면 이 창에서 Ctrl+C. 각 점의 방향은 다음 점을 바라보게 자동으로 정해져요.
"""
import math
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import PointStamped

ROUTES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "routes")


def save(name, pts, frame):
    os.makedirs(ROUTES, exist_ok=True)
    wps = []
    for i, (x, y) in enumerate(pts):
        if i + 1 < len(pts):                     # 다음 점을 바라보는 방향
            nx, ny = pts[i + 1]
            yaw = math.degrees(math.atan2(ny - y, nx - x))
        else:                                    # 마지막 점은 직전 방향 유지
            yaw = wps[-1][2] if wps else 0.0
        wps.append([round(x, 3), round(y, 3), round(yaw, 1)])
    path = os.path.join(ROUTES, f"{name}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"frame": frame, "waypoints": wps}, f, allow_unicode=True, sort_keys=False)
    return path


def main():
    if len(sys.argv) < 2:
        sys.exit("사용법: route.sh record <경로이름>")
    name = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("route_recorder")
    pts, frame = [], ["map"]

    def on_point(msg):
        frame[0] = msg.header.frame_id or "map"
        pts.append((msg.point.x, msg.point.y))
        path = save(name, pts, frame[0])         # 찍을 때마다 저장 (중간에 꺼져도 남음)
        node.get_logger().info(f"{len(pts)}번째 점 ({msg.point.x:.2f}, {msg.point.y:.2f}) → {path}")

    node.create_subscription(PointStamped, "/clicked_point", on_point, 10)
    print(f"📍 경로 '{name}' 기록 중 — RViz에서 'Publish Point'를 누르고 지도를 클릭하세요. 끝나면 Ctrl+C")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if pts:
        print(f"\n✅ 점 {len(pts)}개 저장: {save(name, pts, frame[0])}")
    else:
        print("\n찍은 점이 없어서 저장하지 않았어요.")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

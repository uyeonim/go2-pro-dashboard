"""로봇 내장 SLAM(uslam_server) 사용: 앱/대시보드로 딴 지도 위에서 위치 찾기 · 목표 이동 · 순찰.

명령은 rt/uslam/client_command 에 "모듈/동작/인자..." 문자열 (unitree_ui/docs/slam.md 참고).
응답·상태는 rt/uslam/server_log 문자열. 로봇 위치(지도 좌표)는 rt/uslam/localization/odom.
지도 파일: map.pgm (ROS 방식: 첫 줄 = 위쪽 = y 최대), map.txt ("가로 세로 / 해상도 / 원점x 원점y / 0")
좌표계: 지도 좌표 (m), yaw 는 +x 에서 반시계 rad, [-π, π].
"""
import math
import os
import threading
import time

import cv2
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q):
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))


class RobotSlam:
    def __init__(self, send_cmd, map_dir, log=print):
        self.send_cmd, self.map_dir, self.log = send_cmd, map_dir, log
        self.lock = threading.Lock()
        self.pose = None          # (x, y, yaw, 받은 시각) — 지도 좌표 (위치 찾기 중)
        self.map_pose = None      # 지도 만드는 중의 위치 (frontend/odom)
        self.localized = False
        self.nav_active = False
        self.nav_state = ""
        self.patrol_active = False
        self.patrol_state = ""
        self.mapping = False
        self.goal = None          # (x, y, yaw)
        self.patrol_pts = []
        self.path = []            # 로봇이 계획한 길 [(x, y), ...]
        self.last = []            # 최근 로그
        self.meta = None          # 지도 정보
        self.png = None
        self.load_map()

    # ── 지도 ──
    def load_map(self):
        pgm, txt = os.path.join(self.map_dir, "map.pgm"), os.path.join(self.map_dir, "map.txt")
        if not (os.path.exists(pgm) and os.path.exists(txt)):
            return False
        t = open(pgm, "rb").read().split()
        w, h, mx = int(t[1]), int(t[2]), int(t[3])
        a = np.array(t[4:4 + w * h], dtype=np.int32).reshape(h, w) * 255 // max(1, mx)
        lines = open(txt).read().split()
        res, ox, oy = float(lines[2]), float(lines[3]), float(lines[4])
        rgba = np.zeros((h, w, 4), np.uint8)
        occ = a < 100
        free = a > 200
        mid = ~occ & ~free & (a != 127)
        rgba[occ] = (76, 141, 255, 255)        # 장애물: 파랑 (BGR이 아니라 RGBA로 저장 → 아래서 변환)
        rgba[free] = (60, 72, 92, 255)         # 빈 곳: 어두운 회색
        rgba[mid] = (90, 110, 140, 160)
        bgra = rgba[:, :, [2, 1, 0, 3]]
        self.png = cv2.imencode(".png", bgra)[1].tobytes()
        self.meta = {"w": w, "h": h, "res": res, "ox": ox, "oy": oy, "t": os.path.getmtime(pgm)}
        self.log(f"🗺 로봇 지도 불러옴: {w}x{h}칸, {res}m, 원점 ({ox}, {oy})")
        return True

    # ── 로봇에서 오는 메시지 ──
    def on_log(self, txt):
        txt = str(txt)
        with self.lock:
            self.last = (self.last + [[time.strftime("%H:%M:%S"), txt]])[-8:]
            if "initialization succeed" in txt:
                self.localized = True
            elif "initialization failed" in txt or "localization/stop/success" in txt:
                self.localized = False
            elif "navigation/start/success" in txt:
                self.nav_active = True
            elif "navigation/stop/success" in txt:
                self.nav_active, self.goal, self.path = False, None, []
            elif "navigation/state_transition/" in txt:
                self.nav_state = txt.rsplit("/", 1)[-1]
                if self.nav_state in ("REACHED", "FAILURE", "GOAL_CANCELLED", "NO_PATH"):
                    self.path = []
                    if self.nav_state == "REACHED" and not self.patrol_active:
                        self.goal = None
            elif "patrol/start/success" in txt:
                self.patrol_active = True
            elif "patrol/stop/success" in txt:
                self.patrol_active, self.nav_active, self.path = False, False, []
            elif "patrol/state_transition/" in txt:
                self.patrol_state = txt.rsplit("/", 1)[-1]
            elif "mapping/start/success" in txt:
                self.mapping = True
            elif "mapping/stop/success" in txt:
                self.mapping = False

    def on_loc_odom(self, d):
        try:
            p = d["pose"]["pose"]
            self.pose = (p["position"]["x"], p["position"]["y"], quat_yaw(p["orientation"]), time.time())
        except (KeyError, TypeError):
            pass

    def on_map_odom(self, d):
        try:
            p = d["pose"]["pose"]
            self.map_pose = (p["position"]["x"], p["position"]["y"], quat_yaw(p["orientation"]), time.time())
        except (KeyError, TypeError):
            pass

    def on_path(self, d):
        """global_path: PointCloud2 (JSON 또는 원본 바이트). x,y만 꺼냄 (형식이 다르면 조용히 무시)"""
        try:
            raw = d.get("data")
            fields = {f["name"]: f["offset"] for f in d.get("fields", [])}
            step = int(d.get("point_step", 0))
            if isinstance(raw, list):
                raw = bytes(raw)
            if not (isinstance(raw, (bytes, bytearray)) and step and "x" in fields):
                return
            arr = np.frombuffer(raw, np.uint8)
            n = len(arr) // step
            rows = arr[: n * step].reshape(n, step)
            xs = rows[:, fields["x"]:fields["x"] + 4].copy().view("<f4").ravel()
            ys = rows[:, fields["y"]:fields["y"] + 4].copy().view("<f4").ravel()
            k = max(1, n // 300)
            self.path = [[round(float(x), 2), round(float(y), 2)] for x, y in zip(xs[::k], ys[::k])]
        except Exception:
            pass

    # ── 명령 ──
    def _f(self, v):
        return f"{float(v):.3f}"

    def localize(self, x, y, yaw):
        self.send_cmd(f"localization/set_initial_pose/{self._f(x)}/{self._f(y)}/{self._f(wrap(yaw))}")
        time.sleep(0.15)
        self.send_cmd("localization/start")

    def goto(self, x, y, yaw):
        if not self.nav_active:
            self.send_cmd("navigation/start")
            time.sleep(1.0)
        self.goal = (x, y, wrap(yaw))
        self.send_cmd(f"navigation/set_goal_pose/{self._f(x)}/{self._f(y)}/{self._f(wrap(yaw))}")

    def patrol(self, pts, loops=1):
        """pts: [(x, y, yaw), ...]  loops: 몇 바퀴 (-1 = 계속)"""
        self.patrol_pts = [tuple(p) for p in pts]
        self.send_cmd("patrol/start")                 # 반드시 먼저 (INITIALIZE 상태로)
        time.sleep(0.6)
        self.send_cmd("patrol/clear_all_patrol_points")
        time.sleep(0.2)
        for x, y, yaw in pts:
            self.send_cmd(f"patrol/add_patrol_point/{self._f(x)}/{self._f(y)}/{self._f(wrap(yaw))}")
            time.sleep(0.15)
        self.send_cmd(f"patrol/set_patrol_number_limit/{int(loops)}")
        time.sleep(0.15)
        self.send_cmd("patrol/set_bms_soc_limit/1/99")   # 배터리로 자동 충전하러 가는 것 끔
        time.sleep(0.15)
        self.send_cmd("patrol/go")

    def stop(self):
        self.send_cmd("patrol/stop")
        self.send_cmd("navigation/stop")
        with self.lock:
            self.goal, self.path = None, []

    def busy(self):
        return self.nav_active or self.patrol_active

    def status(self):
        now = time.time()
        p = self.pose if self.pose and now - self.pose[3] < 1.5 else None
        mp = self.map_pose if self.map_pose and now - self.map_pose[3] < 1.5 else None
        with self.lock:
            return {"has_map": self.meta is not None, "map": self.meta,
                    "localized": self.localized and p is not None, "pose": list(p[:3]) if p else None,
                    "map_pose": list(mp[:3]) if mp else None, "mapping": self.mapping,
                    "nav_active": self.nav_active, "nav_state": self.nav_state,
                    "patrol_active": self.patrol_active, "patrol_state": self.patrol_state,
                    "goal": list(self.goal) if self.goal else None,
                    "patrol_pts": [list(q) for q in self.patrol_pts] if self.patrol_active else [],
                    "path": self.path, "log": self.last[-4:]}
